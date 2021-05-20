if [ -d "build" ] 
then
    echo "Directory ./build exists." 
else
    mkdir build
fi

cd build
cmake ..
make -j4