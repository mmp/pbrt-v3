#ifndef IISPTPOINT2I_H
#define IISPTPOINT2I_H

namespace pbrt {

struct IisptPoint2i
{
    int x = 0;
    int y = 0;

    bool operator==(const IisptPoint2i &other) const
    {
        return (x == other.x) && (y == other.y);
    }
};

}

namespace std {

    template <>
    struct hash<pbrt::IisptPoint2i>
    {
        std::size_t operator()(const pbrt::IisptPoint2i& k) const
        {
            return k.x + 777*k.y;
        }
    };

}

#endif // IISPTPOINT2I_H
