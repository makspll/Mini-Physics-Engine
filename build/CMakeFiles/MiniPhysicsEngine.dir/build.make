# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/maks/Git/mini-physics-engine

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maks/Git/mini-physics-engine/build

# Include any dependencies generated for this target.
include CMakeFiles/MiniPhysicsEngine.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MiniPhysicsEngine.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MiniPhysicsEngine.dir/flags.make

CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.o: ../src/AABB.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.o -c /home/maks/Git/mini-physics-engine/src/AABB.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/AABB.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/AABB.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.o: ../src/Fixture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.o -c /home/maks/Git/mini-physics-engine/src/Fixture.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/Fixture.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/Fixture.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.o: ../src/ForceRegistry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.o -c /home/maks/Git/mini-physics-engine/src/ForceRegistry.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/ForceRegistry.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/ForceRegistry.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.o: ../src/MathFunctions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.o -c /home/maks/Git/mini-physics-engine/src/MathFunctions.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/MathFunctions.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/MathFunctions.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.o: ../src/Particle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.o -c /home/maks/Git/mini-physics-engine/src/Particle.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/Particle.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/Particle.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.o: ../src/ParticleContact.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.o -c /home/maks/Git/mini-physics-engine/src/ParticleContact.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/ParticleContact.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/ParticleContact.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.o: ../src/ParticleContactResolver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.o -c /home/maks/Git/mini-physics-engine/src/ParticleContactResolver.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/ParticleContactResolver.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/ParticleContactResolver.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.o: ../src/ParticleForceGenerators.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.o -c /home/maks/Git/mini-physics-engine/src/ParticleForceGenerators.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/ParticleForceGenerators.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/ParticleForceGenerators.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.o: ../src/ParticleForceRegistry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.o -c /home/maks/Git/mini-physics-engine/src/ParticleForceRegistry.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/ParticleForceRegistry.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/ParticleForceRegistry.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.o: ../src/ParticlePhysicsWorld.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.o -c /home/maks/Git/mini-physics-engine/src/ParticlePhysicsWorld.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/ParticlePhysicsWorld.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/ParticlePhysicsWorld.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.o: ../src/PhysicsWorld.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.o -c /home/maks/Git/mini-physics-engine/src/PhysicsWorld.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/PhysicsWorld.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/PhysicsWorld.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.o: ../src/Polygon.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.o -c /home/maks/Git/mini-physics-engine/src/Polygon.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/Polygon.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/Polygon.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.o: ../src/RigidBody.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.o -c /home/maks/Git/mini-physics-engine/src/RigidBody.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/RigidBody.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/RigidBody.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.o: ../src/RigidBodyElastic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.o -c /home/maks/Git/mini-physics-engine/src/RigidBodyElastic.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/RigidBodyElastic.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/RigidBodyElastic.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.o: ../src/RigidBodyFixedElastic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.o -c /home/maks/Git/mini-physics-engine/src/RigidBodyFixedElastic.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/RigidBodyFixedElastic.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/RigidBodyFixedElastic.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.o: ../src/RigidBodyFixedGravity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.o -c /home/maks/Git/mini-physics-engine/src/RigidBodyFixedGravity.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/RigidBodyFixedGravity.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/RigidBodyFixedGravity.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.o: ../src/RigidBodyFixedSpring.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.o -c /home/maks/Git/mini-physics-engine/src/RigidBodyFixedSpring.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/RigidBodyFixedSpring.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/RigidBodyFixedSpring.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.o: ../src/RigidBodyGravity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.o -c /home/maks/Git/mini-physics-engine/src/RigidBodyGravity.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/RigidBodyGravity.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/RigidBodyGravity.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.o: ../src/RigidBodySimpleGravity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.o -c /home/maks/Git/mini-physics-engine/src/RigidBodySimpleGravity.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/RigidBodySimpleGravity.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/RigidBodySimpleGravity.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.o: ../src/RigidBodySpring.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.o -c /home/maks/Git/mini-physics-engine/src/RigidBodySpring.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/RigidBodySpring.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/RigidBodySpring.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.o: ../src/Utility.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.o -c /home/maks/Git/mini-physics-engine/src/Utility.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/Utility.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/Utility.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_22) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.o -c /home/maks/Git/mini-physics-engine/src/main.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/main.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/main.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.s

CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.o: CMakeFiles/MiniPhysicsEngine.dir/flags.make
CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.o: ../src/mainP.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_23) "Building CXX object CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.o -c /home/maks/Git/mini-physics-engine/src/mainP.cpp

CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maks/Git/mini-physics-engine/src/mainP.cpp > CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.i

CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maks/Git/mini-physics-engine/src/mainP.cpp -o CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.s

# Object files for target MiniPhysicsEngine
MiniPhysicsEngine_OBJECTS = \
"CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.o" \
"CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.o"

# External object files for target MiniPhysicsEngine
MiniPhysicsEngine_EXTERNAL_OBJECTS =

MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/AABB.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/Fixture.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/ForceRegistry.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/MathFunctions.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/Particle.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContact.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/ParticleContactResolver.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceGenerators.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/ParticleForceRegistry.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/ParticlePhysicsWorld.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/PhysicsWorld.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/Polygon.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/RigidBody.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyElastic.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedElastic.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedGravity.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyFixedSpring.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodyGravity.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySimpleGravity.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/RigidBodySpring.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/Utility.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/main.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/src/mainP.cpp.o
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/build.make
MiniPhysicsEngine: /usr/lib/x86_64-linux-gnu/libarmadillo.so
MiniPhysicsEngine: /usr/lib/x86_64-linux-gnu/libarmadillo.so
MiniPhysicsEngine: CMakeFiles/MiniPhysicsEngine.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maks/Git/mini-physics-engine/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_24) "Linking CXX executable MiniPhysicsEngine"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MiniPhysicsEngine.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MiniPhysicsEngine.dir/build: MiniPhysicsEngine

.PHONY : CMakeFiles/MiniPhysicsEngine.dir/build

CMakeFiles/MiniPhysicsEngine.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MiniPhysicsEngine.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MiniPhysicsEngine.dir/clean

CMakeFiles/MiniPhysicsEngine.dir/depend:
	cd /home/maks/Git/mini-physics-engine/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maks/Git/mini-physics-engine /home/maks/Git/mini-physics-engine /home/maks/Git/mini-physics-engine/build /home/maks/Git/mini-physics-engine/build /home/maks/Git/mini-physics-engine/build/CMakeFiles/MiniPhysicsEngine.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MiniPhysicsEngine.dir/depend
