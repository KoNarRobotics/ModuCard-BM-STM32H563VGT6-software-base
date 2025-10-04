#!/bin/sh

# Get the number of commits in the repository
minor_version=$(git rev-list --all --count)
major_version=$2


# Define the destination file
dst_file="$1/src/version.hpp"

# read versions from the file
file_major_version=$(cat $dst_file | grep "VERSION_MAJOR" | cut -d' ' -f3)
file_minor_version=$(cat $dst_file | grep "VERSION_MINOR" | cut -d' ' -f3)
file_build_number=$(cat $dst_file | grep "VERSION_BUILD" | cut -d' ' -f3)

# if build number is empty, set it to 0
if [ -z "$file_build_number" ]; then
  build_number=0
fi

# if current minor version is greater than the one in the file, set build number to 0
if [ $minor_version -eq $file_minor_version ]; then
  build_number=$(($file_build_number+1))
else
  build_number=0
fi

echo "Software version: [$major_version.$minor_version.$build_number]"
# Generate the version.hpp file
cat <<EOF > $dst_file
#ifndef VERSION_HPP
#define VERSION_HPP

#define VERSION_MAJOR $major_version
#define VERSION_MINOR $minor_version
#define VERSION_BUILD $build_number

// for easy find  in a .bin file
#define VERSION_DEF_BUILD_STRING_DEF "build_version:$major_version.$minor_version.$build_number"
static const char* VERSION_DEF_BUILD_STRING = VERSION_DEF_BUILD_STRING_DEF;


#endif // VERSION_HPP
EOF