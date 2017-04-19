
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

// core/texcache.cpp*
#include <fcntl.h>
#if defined(PBRT_IS_LINUX) || defined(PBRT_IS_OSX)
#include <sys/time.h>
#include <sys/resource.h>
#endif
#ifdef PBRT_IS_MSVC
#include <sys/types.h>
#include <io.h>
#endif

#include <ratio>
#include <set>
#include <thread>
#include "parallel.h"
#include "stats.h"
#include "texcache.h"

namespace pbrt {

static std::chrono::duration<float, std::milli> minReadDuration(0);

STAT_RATIO("Texture/Tiles read", tilesRead, totalTiles);
STAT_COUNTER("Texture/File open calls", fileOpens);
STAT_PERCENT("Texture/File cache hits", fileCacheHits, fileCacheLookups);
STAT_PERCENT("Texture/Tile cache hits", tileCacheHits, tileCacheLookups);

STAT_MEMORY_COUNTER("Texture/Texels read from disk", texelReadBytes);
STAT_FLOAT_DISTRIBUTION("Texture/Hash table load", hashLoad);
STAT_INT_DISTRIBUTION("Texture/Tile hash probes", tileHashProbes);
STAT_INT_DISTRIBUTION("Texture/MIP level accessed", mipLevelAccessed);
STAT_INT_DISTRIBUTION("Texture/Orphaned texture tiles", orphanedTiles);
STAT_COUNTER("Texture/Tile free passes", freePasses);
STAT_MEMORY_COUNTER("Texture/Top-levels in memory", topLevelBytes);
STAT_FLOAT_DISTRIBUTION("Texture/Tile read time (ms)", tileReadTimeMS);
STAT_COUNTER("Texture/Total read time (ms)", totalTileReadMS);

// Texture Cache Utility Functions
inline Point2i nTiles(Point2i resolution, int logTileSize) {
    int tileSize = 1 << logTileSize;
    return {(resolution[0] + tileSize - 1) >> logTileSize,
            (resolution[1] + tileSize - 1) >> logTileSize};
}

// http://zimbry.blogspot.ch/2011/09/better-bit-mixing-improving-on.html
inline uint64_t MixBits(uint64_t v) {
    v ^= (v >> 31);
    v *= 0x7fb5d329728ea185;
    v ^= (v >> 27);
    v *= 0x81dadef4bc2dd44d;
    v ^= (v >> 33);
    return v;
}

// TileId Declarations
struct TileId {
    // TileId Public Methods
    TileId() : TileId(0, -1, {0, 0}) {}
    TileId(int texId32, int level32, Point2i p32) {
        CHECK(texId32 >= 0 && texId32 < 1 << 16);
        CHECK(level32 >= -1 && level32 < 1 << 15);
        CHECK(p32[0] >= 0 && p32[0] < 1 << 16);
        CHECK(p32[1] >= 0 && p32[1] < 1 << 16);
        bits = texId32;
        bits |= (uint64_t(level32) & 0xffff) << 16;
        bits |= uint64_t(p32[0]) << 32;
        bits |= uint64_t(p32[1]) << 48;
    }
    int texId() const { return bits & 0xffff; }
    int level() const { return (bits >> 16) & 0xffff; }
    Point2i p() const {
        return Point2i((bits >> 32) & 0xffff, (bits >> 48) & 0xffff);
    }
    size_t hash() const { return MixBits(bits); }
    bool operator==(const TileId &t) const { return t.bits == bits; }
    bool operator<(const TileId &t) const { return bits < t.bits; }
    friend inline std::ostream &operator<<(std::ostream &os,
                                           const TileId &tid) {
        return os << StringPrintf("TileId { texId:%d level:%d p:%d, %d }",
                                  tid.texId(), tid.level(), tid.p()[0],
                                  tid.p()[1]);
    }

  private:
    uint64_t bits;
};

// TextureTile Declarations
struct TextureTile {
    // TextureTile Public Methods
    void Clear() {
        tileId = TileId();
        marked.store(false, std::memory_order_relaxed);
    }

    // TextureTile Public Data
    TileId tileId;
    char *texels = nullptr;
    mutable std::atomic<bool> marked;
};

// TiledImagePyramid Method Definitions
bool TiledImagePyramid::Create(std::vector<Image> images,
                               const std::string &filename, WrapMode wrapMode,
                               int tileSize, int topLevelsBytes) {
    FILE *f = fopen(filename.c_str(), "wb");
    if (!f) {
        Error("%s: %s", filename.c_str(), strerror(errno));
        return false;
    }
    // Compute tile size and allocate temporary buffer for tiles
    PixelFormat format = images[0].format;
    // CHECK_LE(tileSize * tileSize * TexelBytes(format), TileAllocSize);
    CHECK(IsPowerOf2(tileSize));
    int logTileSize = Log2Int(tileSize);
    int bufSize = tileSize * tileSize * TexelBytes(format);
    bufSize = (bufSize + TileDiskAlignment - 1) & ~(TileDiskAlignment - 1);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    int64_t filePos;
    int firstInMemoryLevel;
    // Write tiled texture file header
    auto writeInt = [f](int32_t v) {
        return fwrite(&v, sizeof(int), 1, f) == 1;
    };
    if (!writeInt(tiledFileMagic) || !writeInt(int(format)) ||
        !writeInt(int(wrapMode)) || !writeInt(logTileSize) ||
        !writeInt(images.size()))
        goto fail;
    for (size_t level = 0; level < images.size(); ++level)
        if (!writeInt(images[level].resolution[0]) ||
            !writeInt(images[level].resolution[1]))
            goto fail;

    // Write packed top levels of the image pyramid

    // Determine how many top levels to store packed
    firstInMemoryLevel = images.size();
    for (int level = images.size() - 1; level >= 0; --level) {
        int levelBytes = images[level].resolution[0] *
                         images[level].resolution[1] * TexelBytes(format);
        topLevelsBytes -= levelBytes;
        if (topLevelsBytes < 0) break;
        --firstInMemoryLevel;
    }

    // Write out texels for the packed top levels of the pyramid
    if (!writeInt(firstInMemoryLevel)) goto fail;
    for (int level = firstInMemoryLevel; level < images.size(); ++level) {
        int totalPixels =
            images[level].resolution[0] * images[level].resolution[1];
        if (fwrite(images[level].RawPointer({0, 0}), totalPixels,
                   TexelBytes(format), f) != TexelBytes(format))
            goto fail;
    }

    // Advance to the required on-disk alignment before writing tiles
    filePos = ftell(f);
    filePos = (filePos + TileDiskAlignment - 1) & ~(TileDiskAlignment - 1);
    if (fseek(f, filePos, SEEK_SET) != 0) goto fail;

    // Write texture tiles for remaining levels of the pyramid
    for (size_t level = 0; level < firstInMemoryLevel; ++level) {
        const Image &image = images[level];
        CHECK(format == image.format);
        Point2i resolution = image.resolution;
        Bounds2i tileBounds({0, 0}, nTiles(resolution, logTileSize));
        for (Point2i tile : tileBounds) {
            // Write texels for _tile_ to disk
            memset(buf.get(), 0, bufSize);

            // Compute image-space bounds for this tile
            Point2i pMin(tile[0] * tileSize, tile[1] * tileSize);
            Point2i pMax(pMin[0] + tileSize, pMin[1] + tileSize);
            pMax = Min(pMax, resolution);

            // Fill the tile with texel values from the source texture
            for (int y = pMin.y; y < pMax.y; ++y)
                for (int x = pMin.x; x < pMax.x; ++x) {
                    int outOffset = TexelBytes(format) *
                                    ((y - pMin.y) * tileSize + (x - pMin.x));
                    memcpy(&buf[outOffset], image.RawPointer({x, y}),
                           TexelBytes(format));
                }
            if (fwrite(buf.get(), 1, bufSize, f) != bufSize) goto fail;
        }
    }

    // Close tiled texture and return
    if (fclose(f) != 0) {
        Error("%s: %s", filename.c_str(), strerror(errno));
        return false;
    }
    return true;
fail:
    // Handle error case for tiled texture creation
    Error("%s: %s", filename.c_str(), strerror(errno));
    fclose(f);
    return false;
}

bool TiledImagePyramid::Read(const std::string &filename,
                             TiledImagePyramid *tex) {
    FILE *file = fopen(filename.c_str(), "rb");
    if (!file) {
        Error("%s: %s", filename.c_str(), strerror(errno));
        return false;
    }

    auto readInt = [&](int32_t *v) {
        if (fread(v, sizeof(*v), 1, file) != 1) {
            Error("%s: %s", filename.c_str(), strerror(errno));
            fclose(file);
            return false;
        }
        // TODO: handle endian issues
        return true;
    };

    int32_t magic, format, wrap, nLevels, logTileSize;
    if (!readInt(&magic) || !readInt(&format) || !readInt(&wrap) ||
        !readInt(&logTileSize) || !readInt(&nLevels)) {
        Error("%s: %s", filename.c_str(), strerror(errno));
        fclose(file);
        return false;
    }

    if (magic != tiledFileMagic) {
        Error("%s: doesn't appear to be a valid txp file", filename.c_str());
        LOG(ERROR) << StringPrintf("Read magic %x, expected %x", magic,
                                   tiledFileMagic);
        fclose(file);
        return false;
    }

    tex->filename = filename;
    tex->pixelFormat = PixelFormat(format);
    tex->wrapMode = WrapMode(wrap);
    tex->logTileSize = logTileSize;
    for (int i = 0; i < nLevels; ++i) {
        int32_t res[2];
        if (!readInt(&res[0]) || !readInt(&res[1])) {
            Error("%s: %s", filename.c_str(), strerror(errno));
            fclose(file);
            return false;
        }
        tex->levelResolution.push_back(Point2i(res[0], res[1]));
    }

    int32_t firstInMemoryLevel;
    if (!readInt(&firstInMemoryLevel)) {
        Error("%s: %s", filename.c_str(), strerror(errno));
        fclose(file);
        return false;
    }
    tex->firstInMemoryLevel = firstInMemoryLevel;
    for (int level = firstInMemoryLevel; level < nLevels; ++level) {
        int levelBytes = tex->levelResolution[level][0] *
                         tex->levelResolution[level][1] *
                         TexelBytes(tex->pixelFormat);
        topLevelBytes += levelBytes;
        // TODO: single allocation for all of these
        std::unique_ptr<char[]> buf(new char[levelBytes]);
        // TODO: endian
        if (!fread(buf.get(), levelBytes, 1, file)) {
            Error("%s: %s", filename.c_str(), strerror(errno));
            fclose(file);
            return false;
        }
        tex->inMemoryLevels.push_back(std::move(buf));
    }

    int64_t levelPos = ftell(file);
    levelPos = (levelPos + TileDiskAlignment - 1) & ~(TileDiskAlignment - 1);
    CHECK_EQ(0, fseek(file, levelPos, SEEK_SET));

    for (int i = 0; i < firstInMemoryLevel; ++i) {
        tex->levelOffset.push_back(levelPos);
        Point2i nt = nTiles(tex->levelResolution[i], logTileSize);
        levelPos += nt[0] * nt[1] * tex->TileDiskBytes();
        totalTiles += nt[0] * nt[1];
    }

    fclose(file);
    return true;
}

// TileHashTable Declarations
class TileHashTable {
  public:
    // TileHashTable Public Methods
    TileHashTable(size_t size);
    void ReportStats();
    int CountOrphaned();
    const char *Lookup(TileId tileId) const;
    void Insert(TextureTile *entry);
    void MarkEntries();
    void CopyActive(TileHashTable *dest);
    void ReclaimUncopied(std::vector<TextureTile *> *returnedEntries);

  private:
    // TileHashTable Private Data
    std::unique_ptr<std::atomic<TextureTile *>[]> table;
    const size_t size;
    std::vector<bool> hashEntryCopied;
};

// TileHashTable Method Definitions
TileHashTable::TileHashTable(size_t size) : size(size) {
    table.reset(new std::atomic<TextureTile *>[ size ]);
    for (size_t i = 0; i < size; ++i) table[i] = nullptr;
    hashEntryCopied.resize(size);
}

void TileHashTable::ReportStats() {
    int active = 0;
    for (size_t i = 0; i < size; ++i)
        if (table[i].load(std::memory_order_relaxed) != nullptr) ++active;
    ReportValue(hashLoad, float(active) / float(size));
}

int TileHashTable::CountOrphaned() {
    int nOrphaned = 0;
    for (size_t i = 0; i < size; ++i) {
        TextureTile *entry = table[i].load(std::memory_order_relaxed);
        if (entry && entry->marked.load(std::memory_order_relaxed) == false &&
            hashEntryCopied[i] == false)
            ++nOrphaned;
    }
    return nOrphaned;
}

const char *TileHashTable::Lookup(TileId tileId) const {
    int hashOffset = tileId.hash() % size;
    int step = 1;
    int probes = 1;
    for (;;) {
        CHECK(hashOffset >= 0 && hashOffset < size);
        const TextureTile *entry =
            table[hashOffset].load(std::memory_order_acquire);
        // Process _entry_ for hash table lookup
        if (entry == nullptr)
            return nullptr;
        else if (entry->tileId == tileId) {
            // Update _entry_'s _marked_ field after cache hit
            if (entry->marked.load(std::memory_order_relaxed))
                entry->marked.store(false, std::memory_order_relaxed);
            ReportValue(tileHashProbes, probes);
            return entry->texels;
        } else {
            hashOffset += step * step;
            ++step;
            if (hashOffset >= size) hashOffset %= size;
            ++probes;
        }
    }
}

void TileHashTable::Insert(TextureTile *tile) {
    int hashOffset = tile->tileId.hash() % size;
    int probes = 1;
    int step = 1;
    for (;;) {
        // Attempt to insert _tile_ at _hashOffset_
        CHECK(hashOffset >= 0 && hashOffset < size);
        TextureTile *cur = nullptr;
        if (table[hashOffset].compare_exchange_weak(
                cur, tile, std::memory_order_release)) {
            ReportValue(tileHashProbes, probes);
            return;
        }

        // Handle compare--exchange failure for hash table insertion
        if (cur != nullptr) {
            hashOffset += step * step;
            ++step;
            if (hashOffset >= size) hashOffset %= size;
            ++probes;
            CHECK_LT(probes, 100);
        }
    }
}

void TileHashTable::MarkEntries() {
    for (size_t i = 0; i < size; ++i) {
        TextureTile *entry = table[i].load(std::memory_order_acquire);
        if (entry) entry->marked.store(true, std::memory_order_relaxed);
    }
}

void TileHashTable::CopyActive(TileHashTable *dest) {
    int nCopied = 0, nActive = 0;
    // Insert unmarked entries from hash table to _dest_
    for (size_t i = 0; i < size; ++i) {
        hashEntryCopied[i] = false;
        if (TextureTile *entry = table[i].load(std::memory_order_acquire)) {
            // Add _entry_ to _dest_ if unmarked
            ++nActive;
            if (entry->marked.load(std::memory_order_relaxed) == false) {
                hashEntryCopied[i] = true;
                ++nCopied;
                dest->Insert(entry);
            }
        }
    }
    ReportValue(hashLoad, float(nActive) / float(size));
    LOG(INFO) << "Copied " << nCopied << " / " << nActive
              << " to dest TileHashTable";

    // Handle case of all entries copied to _freeHashTable_
    if (nCopied == nActive) {
        LOG(WARNING) << "No entries were marked; everything was copied. "
                        "Freeing arbitrarily.";
        for (size_t i = 0; i < dest->size; ++i)
            dest->table[i].store(nullptr, std::memory_order_relaxed);
        int copyCounter = 0;
        const int copyRate = 5;
        for (size_t i = 0; i < size; ++i) {
            TextureTile *entry = table[i].load(std::memory_order_acquire);
            if (entry) {
                // Either copy _entry_ to _dest_ or free it
                hashEntryCopied[i] = copyCounter++ < copyRate;
                if (hashEntryCopied[i])
                    dest->Insert(entry);
                else
                    copyCounter = 0;
            }
        }
    }
}

void TileHashTable::ReclaimUncopied(std::vector<TextureTile *> *returned) {
    for (size_t i = 0; i < size; ++i) {
        if (TextureTile *entry = table[i].load(std::memory_order_relaxed)) {
            if (!hashEntryCopied[i]) returned->push_back(entry);
            table[i].store(nullptr, std::memory_order_relaxed);
        }
    }
}

// FdCache Method Definitions
FdCache::FdCache(int fdsSpared) {
    // Preallocate _FdEntry_s and initialize free list

    // Determine maximum number of file descriptors, _maxFds_ to use for
    // textures
#if defined(PBRT_IS_LINUX) || defined(PBRT_IS_OSX)
    struct rlimit rlim;
    CHECK_EQ(0, getrlimit(RLIMIT_NOFILE, &rlim)) << strerror(errno);
    LOG(INFO) << "Current limit open files " << rlim.rlim_cur << ", max "
              << rlim.rlim_max;
#ifdef PBRT_IS_OSX
    LOG(INFO) << "Open max " << OPEN_MAX;
    rlim.rlim_cur = OPEN_MAX;
#else
    if (rlim.rlim_max != RLIM_INFINITY) rlim.rlim_cur = rlim.rlim_max;
#endif
    CHECK_EQ(0, setrlimit(RLIMIT_NOFILE, &rlim)) << strerror(errno);
    LOG(INFO) << "Max open files now = " << rlim.rlim_cur;
    int maxFds = rlim.rlim_cur - fdsSpared;
#else
    // TODO: figure out the right thing to do here, especially for windows.
    int maxFds = 500;
#endif
    allocPtr.reset(new FdEntry[maxFds]);
    for (int i = 0; i < maxFds; ++i) {
        FdEntry *entry = &allocPtr[i];
        entry->next = freeList;
        freeList = entry;
    }

    // Allocate hash table for _FdCache_
    int nBuckets = RoundUpPow2(maxFds / 32);
    LOG(INFO) << "Allocating " << nBuckets
              << " buckets in hash table for up to " << maxFds
              << " file descriptors.";
    logBuckets = Log2Int(nBuckets);
    hashTable.resize(nBuckets);
}

FdCache::~FdCache() {
    for (size_t i = 0; i < hashTable.size(); ++i) {
        FdEntry *entry = hashTable[i];
        while (entry) {
#ifdef PBRT_IS_WINDOWS
            CHECK_EQ(0, _close(entry->fd)) << strerror(errno);
#else
            CHECK_EQ(0, close(entry->fd)) << strerror(errno);
#endif
            entry = entry->next;
        }
    }
}

FdEntry *FdCache::Lookup(int fileId, const std::string &filename) {
    int hash = MixBits(fileId) & ((1 << logBuckets) - 1);
    CHECK(hash >= 0 && hash < hashTable.size());
    ++fileCacheLookups;
    mutex.lock();
    // Return fd for _fileId_ from hash table if available
    FdEntry *prev = nullptr;
    FdEntry *cur = hashTable[hash];
    while (cur) {
        if (cur->fileId == fileId && !cur->inUse) {
            // Return available _FdEntry_ for _fileId_
            ++fileCacheHits;
            cur->inUse = true;

            // Move _cur_ to head of list
            if (prev != nullptr) {
                prev->next = cur->next;
                cur->next = hashTable[hash];
                hashTable[hash] = cur;
            }
            mutex.unlock();
            return cur;
        }
        prev = cur;
        cur = cur->next;
    }

    // Find a _FdEntry_ from free list or recycle one
    FdEntry *entry = nullptr;
    if (freeList != nullptr) {
        entry = freeList;
        freeList = freeList->next;
    } else {
        // Recycle a not-currently-used entry in the fd hash table
        for (; entry == nullptr; ++nextVictim) {
            if (nextVictim == hashTable.size()) nextVictim = 0;
            FdEntry *cur = hashTable[nextVictim], *prev = nullptr;
            FdEntry *lastAvailable = nullptr, *lastAvailablePrev = nullptr;
            // Find last entry in list that's not currently in use
            while (cur) {
                if (!cur->inUse) {
                    lastAvailable = cur;
                    lastAvailablePrev = prev;
                }
                prev = cur;
                cur = cur->next;
            }
            if (lastAvailable) {
                // Remove entry from list and initialize _entry_
                entry = lastAvailable;
                if (lastAvailablePrev)
                    lastAvailablePrev->next = lastAvailable->next;
                else {
                    CHECK(hashTable[nextVictim] == lastAvailable);
                    hashTable[nextVictim] = lastAvailable->next;
                }
                ++nextVictim;
            }
        }
    }

    // Add _entry_ to head of list
    entry->next = hashTable[hash];
    hashTable[hash] = entry;
    entry->inUse = true;
    mutex.unlock();
    // Open file to get fd for _filename_
    if (entry->fd >= 0) {
        LOG(INFO) << "Closing fd " << entry->fd;
#ifdef PBRT_IS_WINDOWS
        CHECK_EQ(0, _close(entry->fd)) << strerror(errno);
#else
        CHECK_EQ(0, close(entry->fd)) << strerror(errno);
#endif
    }
    entry->fileId = fileId;
    LOG(INFO) << "Opening file for " << filename;
    ++fileOpens;
#ifdef PBRT_IS_WINDOWS
    entry->fd = _open(filename.c_str(), _O_RDONLY);
#else
    entry->fd = open(filename.c_str(), O_RDONLY);
#endif
    CHECK_NE(entry->fd, -1) << "Couldn't open " << filename << ", "
                            << strerror(errno) << ". Try \"ulimit -n 65536\".";
    return entry;
}

void FdCache::Return(FdEntry *entry) {
    CHECK(entry->inUse);
    std::lock_guard<std::mutex> lock(mutex);
    entry->inUse = false;
}

// TextureCache Method Definitions
TextureCache::TextureCache() {
    minReadDuration =
        std::chrono::duration<float, std::milli>(PbrtOptions.texReadMinMS);
    LOG(INFO) << "Min tex read duration " << minReadDuration.count() << "ms";
    // Allocate texture tiles and initialize free list
    size_t maxTextureBytes = size_t(PbrtOptions.texCacheMB) * 1024 * 1024;
    size_t nTiles = maxTextureBytes / TileAllocSize;

    // Allocate tile memory for texture cache
    tileMemAlloc.reset(new char[nTiles * TileAllocSize]);
    char *tilePtr = tileMemAlloc.get();
    LOG(INFO) << "Allocating " << nTiles << " TextureTile objects";

    // Allocate _TextureTile_s and initialize free list
    allTilesAlloc.reset(new TextureTile[nTiles]);
    for (int i = 0; i < nTiles; ++i) {
        allTilesAlloc[i].texels = tilePtr + i * TileAllocSize;
        freeTiles.push_back(&allTilesAlloc[i]);
    }

    // Allocate hash tables for texture tiles
    int hashSize = 8 * nTiles;
    hashTable = new TileHashTable(hashSize);
    freeHashTable = new TileHashTable(hashSize);

    // Initialize _markFreeCapacity_
    markFreeCapacity = 1 + nTiles / 8;

    // Allocate _threadActiveFlags_
    threadActiveFlags = std::vector<ActiveFlag>(MaxThreadIndex());
    if (minReadDuration > std::chrono::duration<float, std::milli>::zero())
        LOG(WARNING)
            << "Will sleep to ensure tile I/O takes at least "
            << std::chrono::duration<float, std::milli>(minReadDuration).count()
            << "ms";
}

TextureCache::~TextureCache() {
    hashTable.load()->ReportStats();
    delete hashTable.load();
    delete freeHashTable;
}

int TextureCache::TileSize(PixelFormat format) {
    switch (format) {
    case PixelFormat::SY8:
    case PixelFormat::Y8:
        return 64;
    case PixelFormat::RGB8:
    case PixelFormat::SRGB8:
        return 64;
    case PixelFormat::Y16:
        return 64;
    case PixelFormat::RGB16:
        return 32;
    case PixelFormat::Y32:
        return 32;
    case PixelFormat::RGB32:
        return 16;
    default:
        LOG(FATAL) << "Unhandled pixel format";
    }
}

int TextureCache::AddTexture(const std::string &filename) {
    // Return preexisting id if texture has already been added
    auto iter = std::find_if(textures.begin(), textures.end(),
                             [&filename](const TiledImagePyramid &tex) {
                                 return tex.filename == filename;
                             });
    if (iter != textures.end()) return iter - textures.begin();
    TiledImagePyramid tex;
    if (!TiledImagePyramid::Read(filename, &tex)) return -1;
    textures.push_back(std::move(tex));
    return textures.size() - 1;
}

void TextureCache::PreloadTexture(int texId) {
    CHECK(texId >= 0 && texId < textures.size());
    const TiledImagePyramid &tex = textures[texId];
    int nLevels = tex.levelResolution.size();
    int tileSize = 1 << tex.logTileSize;

    for (int level = 0; level < nLevels; ++level) {
        Point2i res = tex.levelResolution[level];
        for (int y = 0; y < res.y; y += tileSize)
            for (int x = 0; x < res.x; x += tileSize) {
                (void)GetTile(TileId(texId, level, tex.TileIndex({x, y})));
            }
    }
}

template <typename T>
T TextureCache::Texel(int texId, int level, Point2i p) {
    ReportValue(mipLevelAccessed, level);
    ProfilePhase _(Prof::TexCacheGetTexel);
    CHECK(texId >= 0 && texId < textures.size());
    const TiledImagePyramid &tex = textures[texId];
    Point2i res = tex.levelResolution[level];
    CHECK(p.x >= 0 && p.x < res.x);
    CHECK(p.y >= 0 && p.y < res.y);
    // Return texel from preloaded levels, if applicable
    const char *texel = tex.GetTexel(level, p);
    if (texel != nullptr) return ConvertTexel<T>(texel, tex.pixelFormat);

    // Get texel pointer from cache and return value
    TileId tileId(texId, level, tex.TileIndex(p));
    texel = GetTile(tileId) + tex.TexelOffset(p);
    T ret = ConvertTexel<T>(texel, tex.pixelFormat);
    RCUEnd();
    return ret;
}

// Explicit instantiation
template Float TextureCache::Texel(int texId, int level, Point2i p);
template Spectrum TextureCache::Texel(int texId, int level, Point2i p);

const char *TextureCache::GetTile(TileId tileId) {
    ProfilePhase _(Prof::TexCacheGetTile);
    ++tileCacheLookups;
    // Return tile if it's present in the hash table
    RCUBegin();
    TileHashTable *t = hashTable.load(std::memory_order_acquire);
    if (const char *texels = t->Lookup(tileId)) {
        ++tileCacheHits;
        return texels;
    }
    RCUEnd();

    // Check to see if another thread is already loading this tile
    outstandingReadsMutex.lock();
    for (const TileId &readTileId : outstandingReads) {
        if (readTileId == tileId) {
            // Wait for _tileId_ to be read before retrying lookup
            LOG(INFO) << "Another reader is already on it. Waiting for "
                      << tileId;
            std::unique_lock<std::mutex> readsLock(outstandingReadsMutex,
                                                   std::adopt_lock);
            outstandingReadsCondition.wait(readsLock);
            LOG(INFO) << "Read done. Retrying " << tileId;
            readsLock.unlock();
            return GetTile(tileId);
        }
    }

    // Record that the current thread will read _tileId_
    outstandingReads.push_back(tileId);
    outstandingReadsMutex.unlock();

    // Load texture tile from disk
    TextureTile *tile = GetFreeTile();
    ReadTile(tileId, tile);

    // Add tile to hash table and return texel pointer
    RCUBegin();
    t = hashTable.load(std::memory_order_relaxed);
    t->Insert(tile);

    // Update _outstandingReads_ for read tile
    outstandingReadsMutex.lock();
    DCHECK(std::find(outstandingReads.begin(), outstandingReads.end(),
                     tileId) != outstandingReads.end());
    for (auto iter = outstandingReads.begin(); iter != outstandingReads.end();
         ++iter) {
        if (*iter == tileId) {
            outstandingReads.erase(iter);
            break;
        }
    }
    CHECK_LE(outstandingReads.size(), threadActiveFlags.size());
    outstandingReadsMutex.unlock();
    outstandingReadsCondition.notify_all();
    return tile->texels;
}

TextureTile *TextureCache::GetFreeTile() {
    std::lock_guard<std::mutex> lock(freeTilesMutex);
    if (freeTiles.size() == 0) {
        LOG(INFO) << "Kicking off free";
        FreeTiles();
        CHECK_GT(freeTiles.size(), 0);
    }
    // Mark hash table entries if free-tile availability is low
    if (freeTiles.size() == markFreeCapacity)
        hashTable.load(std::memory_order_acquire)->MarkEntries();

    // Return tile from _freeTiles_
    TextureTile *tile = freeTiles.back();
    freeTiles.pop_back();
    tile->Clear();
    return tile;
}

void TextureCache::ReadTile(TileId tileId, TextureTile *tile) {
    // Note that profiling ReadTile() using the profiling system isn't accurate:
    // signal delivery is disabled when a thread is in the kernel, and most
    // of the time here is in read()...
    ProfilePhase _(Prof::TexCacheReadTile);
    auto startTime = std::chrono::system_clock::now();
    tile->tileId = tileId;
    CHECK(tileId.texId() >= 0 && tileId.texId() < textures.size());
    const TiledImagePyramid &tex = textures[tileId.texId()];
    // Get file descriptor and seek to start of texture tile
    int64_t offset = tex.FileOffset(tileId.level(), tileId.p());
    CHECK_EQ(0, offset & (TileDiskAlignment - 1)) << offset;
    FdEntry *fdEntry = fdCache.Lookup(tileId.texId(), tex.filename);
#ifdef PBRT_IS_WINDOWS
    if (_lseeki64(fdEntry->fd, offset, SEEK_SET) == -1)
#else
    if (lseek(fdEntry->fd, offset, SEEK_SET) == -1)
#endif
        Error("%s: seek error %s", tex.filename.c_str(), strerror(errno));

    // Read texel data and return file descriptor
    int tileBytes = tex.TileBytes();
    CHECK_LE(tileBytes, TileAllocSize);
    Point2i nt = nTiles(tex.levelResolution[tileId.level()], tex.logTileSize);
    CHECK(tileId.p()[0] < nt[0]);
    CHECK(tileId.p()[1] < nt[1]);
    ++tilesRead;
#ifdef PBRT_IS_WINDOWS
    if (_read(fdEntry->fd, tile->texels, tileBytes) != tileBytes)
#else
    if (read(fdEntry->fd, tile->texels, tileBytes) != tileBytes)
#endif
        Error("%s: read error %s", tex.filename.c_str(), strerror(errno));

    using DurationMS = std::chrono::duration<float, std::milli>;
    DurationMS elapsed(std::chrono::system_clock::now() - startTime);
    if (elapsed < minReadDuration)
        std::this_thread::sleep_for(minReadDuration - elapsed);
    elapsed = DurationMS(std::chrono::system_clock::now() - startTime);
    ReportValue(tileReadTimeMS, elapsed.count());
    totalTileReadMS += elapsed.count();

    fdCache.Return(fdEntry);
    texelReadBytes += tileBytes;
}

void TextureCache::FreeTiles() {
    ProfilePhase _(Prof::TexCacheFree);
    LOG(INFO) << "Starting to free memory";
    ++freePasses;
    // Copy unmarked tiles to _freeHashTable_
    hashTable.load(std::memory_order_relaxed)->CopyActive(freeHashTable);

    // Swap texture cache hash tables
    freeHashTable =
        hashTable.exchange(freeHashTable, std::memory_order_acq_rel);

    // Ensure that no threads are accessing the old hash table
    for (size_t i = 0; i < threadActiveFlags.size(); ++i) WaitForQuiescent(i);
    LOG(INFO) << "Got all thread locks";

    // FIXME?: Anything added to the original hash table during
    // CopyActive() and getting the per-thread locks will have its copied
    // field still false; as such it suffers the unlucky fate of being
    // freed shortly after creation here...
    int nOrphaned = freeHashTable->CountOrphaned();
    LOG(INFO) << "Orphaned entries in hash table: " << nOrphaned;
    ReportValue(orphanedTiles, nOrphaned);
    // Add inactive tiles in _freeHashTable_ to free list
    freeHashTable->ReclaimUncopied(&freeTiles);
    if (freeTiles.size() < markFreeCapacity)
        hashTable.load(std::memory_order_acquire)->MarkEntries();
    LOG(INFO) << "Finished freeing";
}

Image TextureCache::GetLevelImage(int texId, int level) {
    Point2i res = textures[texId].levelResolution[level];
    PixelFormat format = textures[texId].pixelFormat;

    Image image(format, res);
    for (int y = 0; y < res.y; ++y)
        for (int x = 0; x < res.x; ++x) {
            if (nChannels(format) == 1)
                image.SetChannel({x, y}, 0, Texel<Float>(texId, level, {x, y}));
            else {
                CHECK_EQ(3, nChannels(format));
                Spectrum s = Texel<Spectrum>(texId, level, {x, y});
                image.SetSpectrum({x, y}, s);
            }
        }

    return image;
}

}  // namespace pbrt
