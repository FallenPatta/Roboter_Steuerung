# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/david/Documents/Projects/Cpp/PrinterLib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/david/Documents/Projects/Cpp/PrinterLib/build

# Include any dependencies generated for this target.
include CMakeFiles/printlibrary.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/printlibrary.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/printlibrary.dir/flags.make

CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o: CMakeFiles/printlibrary.dir/flags.make
CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o: ../src1/MessagePrinter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/Documents/Projects/Cpp/PrinterLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o -c /home/david/Documents/Projects/Cpp/PrinterLib/src1/MessagePrinter.cpp

CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/Documents/Projects/Cpp/PrinterLib/src1/MessagePrinter.cpp > CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.i

CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/Documents/Projects/Cpp/PrinterLib/src1/MessagePrinter.cpp -o CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.s

CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o.requires:

.PHONY : CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o.requires

CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o.provides: CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o.requires
	$(MAKE) -f CMakeFiles/printlibrary.dir/build.make CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o.provides.build
.PHONY : CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o.provides

CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o.provides.build: CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o


CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o: CMakeFiles/printlibrary.dir/flags.make
CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o: ../src1/HelloWorldPrinter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/Documents/Projects/Cpp/PrinterLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o -c /home/david/Documents/Projects/Cpp/PrinterLib/src1/HelloWorldPrinter.cpp

CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/Documents/Projects/Cpp/PrinterLib/src1/HelloWorldPrinter.cpp > CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.i

CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/Documents/Projects/Cpp/PrinterLib/src1/HelloWorldPrinter.cpp -o CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.s

CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o.requires:

.PHONY : CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o.requires

CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o.provides: CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o.requires
	$(MAKE) -f CMakeFiles/printlibrary.dir/build.make CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o.provides.build
.PHONY : CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o.provides

CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o.provides.build: CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o


CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o: CMakeFiles/printlibrary.dir/flags.make
CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o: ../src1/GoodbyeWorldPrinter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/Documents/Projects/Cpp/PrinterLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o -c /home/david/Documents/Projects/Cpp/PrinterLib/src1/GoodbyeWorldPrinter.cpp

CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/Documents/Projects/Cpp/PrinterLib/src1/GoodbyeWorldPrinter.cpp > CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.i

CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/Documents/Projects/Cpp/PrinterLib/src1/GoodbyeWorldPrinter.cpp -o CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.s

CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o.requires:

.PHONY : CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o.requires

CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o.provides: CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o.requires
	$(MAKE) -f CMakeFiles/printlibrary.dir/build.make CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o.provides.build
.PHONY : CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o.provides

CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o.provides.build: CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o


CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o: CMakeFiles/printlibrary.dir/flags.make
CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o: ../src1/PrinterComposite.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/Documents/Projects/Cpp/PrinterLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o -c /home/david/Documents/Projects/Cpp/PrinterLib/src1/PrinterComposite.cpp

CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/Documents/Projects/Cpp/PrinterLib/src1/PrinterComposite.cpp > CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.i

CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/Documents/Projects/Cpp/PrinterLib/src1/PrinterComposite.cpp -o CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.s

CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o.requires:

.PHONY : CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o.requires

CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o.provides: CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o.requires
	$(MAKE) -f CMakeFiles/printlibrary.dir/build.make CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o.provides.build
.PHONY : CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o.provides

CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o.provides.build: CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o


CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o: CMakeFiles/printlibrary.dir/flags.make
CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o: ../src1/DeallocatingPrinterComposite.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/Documents/Projects/Cpp/PrinterLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o -c /home/david/Documents/Projects/Cpp/PrinterLib/src1/DeallocatingPrinterComposite.cpp

CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/Documents/Projects/Cpp/PrinterLib/src1/DeallocatingPrinterComposite.cpp > CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.i

CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/Documents/Projects/Cpp/PrinterLib/src1/DeallocatingPrinterComposite.cpp -o CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.s

CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o.requires:

.PHONY : CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o.requires

CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o.provides: CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o.requires
	$(MAKE) -f CMakeFiles/printlibrary.dir/build.make CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o.provides.build
.PHONY : CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o.provides

CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o.provides.build: CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o


CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o: CMakeFiles/printlibrary.dir/flags.make
CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o: ../src1/PluginProject.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/david/Documents/Projects/Cpp/PrinterLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o -c /home/david/Documents/Projects/Cpp/PrinterLib/src1/PluginProject.cpp

CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/david/Documents/Projects/Cpp/PrinterLib/src1/PluginProject.cpp > CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.i

CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/david/Documents/Projects/Cpp/PrinterLib/src1/PluginProject.cpp -o CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.s

CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o.requires:

.PHONY : CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o.requires

CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o.provides: CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o.requires
	$(MAKE) -f CMakeFiles/printlibrary.dir/build.make CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o.provides.build
.PHONY : CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o.provides

CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o.provides.build: CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o


# Object files for target printlibrary
printlibrary_OBJECTS = \
"CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o" \
"CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o" \
"CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o" \
"CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o" \
"CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o" \
"CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o"

# External object files for target printlibrary
printlibrary_EXTERNAL_OBJECTS =

libprintlibrary.so: CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o
libprintlibrary.so: CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o
libprintlibrary.so: CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o
libprintlibrary.so: CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o
libprintlibrary.so: CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o
libprintlibrary.so: CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o
libprintlibrary.so: CMakeFiles/printlibrary.dir/build.make
libprintlibrary.so: CMakeFiles/printlibrary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/david/Documents/Projects/Cpp/PrinterLib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library libprintlibrary.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/printlibrary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/printlibrary.dir/build: libprintlibrary.so

.PHONY : CMakeFiles/printlibrary.dir/build

CMakeFiles/printlibrary.dir/requires: CMakeFiles/printlibrary.dir/src1/MessagePrinter.cpp.o.requires
CMakeFiles/printlibrary.dir/requires: CMakeFiles/printlibrary.dir/src1/HelloWorldPrinter.cpp.o.requires
CMakeFiles/printlibrary.dir/requires: CMakeFiles/printlibrary.dir/src1/GoodbyeWorldPrinter.cpp.o.requires
CMakeFiles/printlibrary.dir/requires: CMakeFiles/printlibrary.dir/src1/PrinterComposite.cpp.o.requires
CMakeFiles/printlibrary.dir/requires: CMakeFiles/printlibrary.dir/src1/DeallocatingPrinterComposite.cpp.o.requires
CMakeFiles/printlibrary.dir/requires: CMakeFiles/printlibrary.dir/src1/PluginProject.cpp.o.requires

.PHONY : CMakeFiles/printlibrary.dir/requires

CMakeFiles/printlibrary.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/printlibrary.dir/cmake_clean.cmake
.PHONY : CMakeFiles/printlibrary.dir/clean

CMakeFiles/printlibrary.dir/depend:
	cd /home/david/Documents/Projects/Cpp/PrinterLib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/david/Documents/Projects/Cpp/PrinterLib /home/david/Documents/Projects/Cpp/PrinterLib /home/david/Documents/Projects/Cpp/PrinterLib/build /home/david/Documents/Projects/Cpp/PrinterLib/build /home/david/Documents/Projects/Cpp/PrinterLib/build/CMakeFiles/printlibrary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/printlibrary.dir/depend
