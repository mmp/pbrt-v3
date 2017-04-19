
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_TEXCACHE_H
#define PBRT_CORE_TEXCACHE_H

// core/texcache.h*
#include <cstddef>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include "geometry.h"
#include "image.h"
#include "memory.h"
#include "parallel.h"
#include "pbrt.h"

/*
TODO (future);
- allow non-square tiles (but still pow2)

- reduce rgb->y if greyscale, etc.
   - reduce to lower bit depths if appropriate
   - do at Image::Read time, so all benefit?
*/

namespace pbrt {

struct TileId;
struct TextureTile;
class TileHashTable;

// Texture Cache Constants
static const uint32_t tiledFileMagic = 0x65028088;
static PBRT_CONSTEXPR int TileDiskAlignment = 4096;
#ifdef PBRT_IS_LINUX
static PBRT_CONSTEXPR int MaxOpenFiles = 4000;
#else
static PBRT_CONSTEXPR int MaxOpenFiles = 200;  // TODO: how big?
#endif

// ActiveFlag Declarations
#ifdef PBRT_HAVE_ALIGNAS
struct alignas(PBRT_L1_CACHE_LINE_SIZE) ActiveFlag {
#else
struct ActiveFlag { char pad[PBRT_L1_CACHE_LINE_SIZE];
#endif
    std::atomic<bool> flag{false};
};

// TiledImagePyramid Declarations
class TiledImagePyramid {
  public:
    // TiledImagePyramid Public Methods
    static bool Create(std::vector<Image> levels, const std::string &filename,
                       WrapMode wrapMode, int tileSize,
                       int topLevelsBytes = 3800);
    static bool Read(const std::string &filename, TiledImagePyramid *tex);
    size_t TileBytes() const {
        return (1 << logTileSize) * (1 << logTileSize) *
               TexelBytes(pixelFormat);
    }
    size_t TileDiskBytes() const {
        return (TileBytes() + TileDiskAlignment - 1) & ~(TileDiskAlignment - 1);
    }
    Point2i TileIndex(Point2i p) const {
        return {p[0] >> logTileSize, p[1] >> logTileSize};
    }
    int TexelOffset(Point2i p) const {
        int tileMask = (1 << logTileSize) - 1;
        Point2i tilep{p[0] & tileMask, p[1] & tileMask};
        int tileWidth = 1 << logTileSize;
        return TexelBytes(pixelFormat) * (tilep[1] * tileWidth + tilep[0]);
    }
    int64_t FileOffset(int level, Point2i p) const {
        int tileWidth = 1 << logTileSize;
        CHECK_LE(level, levelOffset.size());  // preloaded
        int xTiles = (levelResolution[level][0] + tileWidth - 1) >> logTileSize;
        return levelOffset[level] + TileDiskBytes() * (p[1] * xTiles + p[0]);
    }
    const char *GetTexel(int level, Point2i p) const {
        // Assumes that p has already gone through remapping.
        CHECK(p.x >= 0 && p.x < levelResolution[level].x);
        CHECK(p.y >= 0 && p.y < levelResolution[level].y);

        if (level < firstInMemoryLevel) return nullptr;
        const char *levelStart =
            inMemoryLevels[level - firstInMemoryLevel].get();
        return levelStart +
               TexelBytes(pixelFormat) *
                   (p[1] * levelResolution[level][0] + p[0]);
    }

    // TiledImagePyramid Public Data
    std::string filename;
    PixelFormat pixelFormat;
    WrapMode wrapMode;
    int logTileSize;
    std::vector<Point2i> levelResolution;
    std::vector<int64_t> levelOffset;
    int firstInMemoryLevel;
    std::vector<std::unique_ptr<char[]>> inMemoryLevels;
};

// FdEntry Declarations
class FdEntry {
  public:
    int fd = -1;

  private:
    // FdEntry Private Data
    friend class FdCache;
    int fileId = -1;
    FdEntry *next = nullptr;
    bool inUse = false;
};

// FdCache Declarations
class FdCache {
  public:
    // FdCache Public Methods
    FdCache(int fdsSpared = 40);
    ~FdCache();
    FdEntry *Lookup(int id, const std::string &filename);
    void Return(FdEntry *entry);

  private:
    // FdCache Private Data
    std::unique_ptr<FdEntry[]> allocPtr;
    FdEntry *freeList = nullptr;
    std::vector<FdEntry *> hashTable;
    int logBuckets;
    std::mutex mutex;
    int nextVictim = 0;
};

// TextureCache Declarations
class TextureCache {
  public:
    // TextureCache Public Methods
    TextureCache();
    ~TextureCache();
    static int TileSize(PixelFormat format);
    int AddTexture(const std::string &filename);
    const std::vector<Point2i> &GetLevelResolution(int texId) const {
        CHECK(texId >= 0 && texId < textures.size());
        return textures[texId].levelResolution;
    }
    Point2i GetLevelResolution(int texId, int level) const {
        const std::vector<Point2i> &res = GetLevelResolution(texId);
        CHECK(level >= 0 && level < res.size());
        return res[level];
    }
    WrapMode GetWrapMode(int texId) const {
        CHECK(texId >= 0 && texId < textures.size());
        return textures[texId].wrapMode;
    }
    int Levels(int texId) const {
        CHECK(texId >= 0 && texId < textures.size());
        return textures[texId].levelResolution.size();
    }
    PixelFormat GetPixelFormat(int texId) const {
        CHECK(texId >= 0 && texId < textures.size());
        return textures[texId].pixelFormat;
    }
    void PreloadTexture(int texId);
    template <typename T>
    T Texel(int texId, int level, Point2i p);
    Image GetLevelImage(int texId, int level);

  private:
    // TextureCache Private Methods
    void RCUBegin() {
        std::atomic<bool> &flag = threadActiveFlags[ThreadIndex].flag;
        flag.store(true, std::memory_order_acquire);
    }
    void RCUEnd() {
        std::atomic<bool> &flag = threadActiveFlags[ThreadIndex].flag;
        flag.store(false, std::memory_order_release);
    }
    void WaitForQuiescent(int thread) {
        std::atomic<bool> &flag = threadActiveFlags[thread].flag;
        while (flag.load(std::memory_order_acquire) == true)
            ;  // spin
    }
    const char *GetTile(TileId tileId);
    TextureTile *GetFreeTile();
    void ReadTile(TileId tileId, TextureTile *tile);
    void FreeTiles();

    // TextureCache Private Data
    static PBRT_CONSTEXPR int TileAllocSize = 3 * 64 * 64;
    std::unique_ptr<char[]> tileMemAlloc;
    std::unique_ptr<TextureTile[]> allTilesAlloc;
    std::mutex freeTilesMutex;
    std::vector<TextureTile *> freeTiles;
    std::atomic<TileHashTable *> hashTable;
    TileHashTable *freeHashTable;
    std::vector<TiledImagePyramid> textures;
    std::vector<ActiveFlag> threadActiveFlags;

#ifdef PBRT_HAVE_ALIGNAS
    alignas(PBRT_L1_CACHE_LINE_SIZE)
#else
    char pad[PBRT_L1_CACHE_LINE_SIZE];
#endif
    std::mutex outstandingReadsMutex;

    std::vector<TileId> outstandingReads;
    std::condition_variable outstandingReadsCondition;
    FdCache fdCache;
    int markFreeCapacity;
};

}  // namespace pbrt

#endif  // PBRT_CORE_TEXCACHE_H
