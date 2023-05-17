if [ ! -d ./results/traditional/ ]; then
    mkdir -p ./results/traditional/
fi

if [ ! -d ./results/guided/ ]; then
    mkdir -p ./results/guided/
fi

for((i=0;i<20;i++)); do
    ./build/pbrt ./scenes/guide.pbrt
    mv ./guide_bunny.exr ./results/guided/bunny-s200-d256-${i}.exr
done
