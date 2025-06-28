# ask for full recompilation(clean) or incremental recompilation
#!/bin/bash
echo "Do you want to perform a full clean build? (y/n) or just run(r)"
read clean_choice
if [ "$clean_choice" = "r" ]; then
    echo "Running the existing build..."
    if [ -f sasu_icar2025_demo/src/build/icar ]; then
        cd sasu_icar2025_demo/src/build
        ./icar
        exit 0
    else
        echo "No existing build found, proceeding with clean build."
        clean_choice="y"
    fi
fi

if [ "$clean_choice" = "y" ]; then
    echo "Performing full clean build..."
    rm -rf sasu_icar2025_demo/src/build
else
    echo "Performing incremental build..."
fi

mkdir -p sasu_icar2025_demo/src/build
cd sasu_icar2025_demo/src/build
cmake ..
make -j4

if [ $? -eq 0 ]; then
    cd ../
    cp -f build/icar icar

    echo "Build successful, run programme?(y/n)"
    read run_choice
    if [ "$run_choice" = "y" ]; then
        ./icar
    else
        echo "Skipping execution."
    fi
else
    echo "Build failed, please check the output for errors."
fi