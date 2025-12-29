# CMake generated Testfile for 
# Source directory: /home/joe/ai/ai-sdr-lora/examples/c
# Build directory: /home/joe/ai/ai-sdr-lora/examples/c/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(fft_demo_run "/home/joe/ai/ai-sdr-lora/examples/c/build/fft_demo")
set_tests_properties(fft_demo_run PROPERTIES  ENVIRONMENT "LD_LIBRARY_PATH=/home/joe/ai/ai-sdr-lora/examples/c/../../target/release" WORKING_DIRECTORY "/home/joe/ai/ai-sdr-lora/examples/c/build" _BACKTRACE_TRIPLES "/home/joe/ai/ai-sdr-lora/examples/c/CMakeLists.txt;92;add_test;/home/joe/ai/ai-sdr-lora/examples/c/CMakeLists.txt;0;")
