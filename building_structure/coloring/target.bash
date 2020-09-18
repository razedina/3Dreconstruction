rm -r build # 
mkdir build #
cd build 
cmake .. #
make 

./target
cd ..
# /TheiaSfM/build/bin/write_reconstruction_ply_file --reconstruction=/TheiaSfM/iron/iron_mapping --ply_file=/TheiaSfM/iron/iron_mapping.ply

#meshlab project/mapping.ply
