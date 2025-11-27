#!/bin/bash

# Prerequisites:
# sudo apt install qhull-bin
# sudo apt install assimp-utils

# refering to https://github.com/isri-aist/mc_fetch_description/blob/main/scripts/generate_convex.sh

# To visualize the generated convex hulls with rviz, you can use: 
# 'mc_convex_visualization ROBOTNAME' command from mc_rtc package.

# The convex must have the same name as the link name of the robot, with suffix '-ch.txt'.
# Thus it can be different than the original mesh file name.

exit_if_error()
{
  if [ $? -ne 0 ]
  then
    echo "-- FATAL ERROR: $1"
    exit 1
  fi
}

# set configuration variables
export robot_name="kinova"
export current_path="$(pwd)"                # You must run this script from the mc_kinova root directory
export sample_points=2000                   # Number of points to sample on each mesh (used for convex hull generation)

echo "Running generate_convex.sh script from directory `pwd`"

function generate_convexes()
{
    # List target mesh files
    stlfiles=`find ${current_path}/meshes/ -type f -regex ".*STL$"`
    targets="${daefiles} ${stlfiles}"
    echo ${targets}

    # Generate convexes (convert to qhull's pointcloud and compute convex hull file)
    mkdir -p ${current_path}/convex/${robot_name}
    mkdir -p ${current_path}/qc/${robot_name}
    for mesh in ${targets}
    do
        echo "-- Processing mesh ${mesh}"
        mesh_name=`basename -- "$mesh"`
        mesh_name="${mesh_name%.*}"
        echo "-- Generating convex hull for ${mesh}"
        gen_convex=${current_path}/convex/${robot_name}/${mesh_name}-ch.txt
        mesh_sampling --in "${mesh}" --convex "${current_path}/convex/${robot_name}/" --type xyz --samples "${sample_points}"
        exit_if_error "Failed to sample pointcloud from mesh ${mesh}"
    done
}

generate_convexes

echo
echo "Successfully generated convex in ${current_path}/convex/${robot_name}"

