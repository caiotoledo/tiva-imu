cmake_minimum_required(VERSION 3.10)
set(PROGRAM_NAME timer)

file(GLOB_RECURSE src_files "./*.cpp")
file(GLOB src_c "./../src/*.c")

package_gtest(${PROGRAM_NAME}
		SRC
			${src_files} ${src_c}
		INCLUDE
			./include/ ./../include/
)
