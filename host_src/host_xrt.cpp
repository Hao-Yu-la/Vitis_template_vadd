// XRT API 

#include <getopt.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cstring>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <assert.h>
#include <stdint.h>
#include "xclhal2.h"
#include "xclbin.h"

// XRT includes
#include "xrt_bo.h"
#include "xrt_device.h"
#include "xrt_kernel.h"

// driver includes
#include "ert.h"

#define DATA_SIZE 4096

#define NUM_WORKGROUPS (1)
#define WORKGROUP_SIZE (256)
#define MAX_LENGTH 8192
#define MEM_ALIGNMENT 4096
#if defined(VITIS_PLATFORM) && !defined(TARGET_DEVICE)
#define STR_VALUE(arg)      #arg
#define GET_STRING(name) STR_VALUE(name)
#define TARGET_DEVICE GET_STRING(VITIS_PLATFORM)
#endif

uint32_t load_file_to_memory(const char *filename, char **result)
{
    uint32_t size = 0;
    FILE *f = fopen(filename, "rb");
    if (f == NULL) {
        *result = NULL;
        return -1; // -1 means file opening fail
    }
    fseek(f, 0, SEEK_END);
    size = ftell(f);
    fseek(f, 0, SEEK_SET);
    *result = (char *)malloc(size+1);
    if (size != fread(*result, sizeof(char), size, f)) {
        free(*result);
        return -2; // -2 means file reading fail
    }
    fclose(f);
    (*result)[size] = 0;
    return size;
}

int main(int argc, char** argv)
{

     if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <XCLBIN File>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string binaryFile = argv[1];
    int device_index = 0; //which FPGA device 0 or 1

    //Open the device
    std::cout << "Open the device " << device_index << std::endl;
    auto device = xrt::device(device_index);
    auto device_name = device.get_info<xrt::info::device::name>();
    char target_device_name[1001] = TARGET_DEVICE;
    if (device_name.find(target_device_name) != std::string::npos) {
        std::cout << "[INFO]: The example runs on non-CDMA mode for " << device_name << '\n';
    }
    else{
        std::cout << "Opened the device " << device_name << '\n';
    }

    //Load the xclbin
    std::cout << "Load the xclbin " << binaryFile << std::endl;
    auto uuid = device.load_xclbin(binaryFile);
    std::cout << "Loaded the xclbin " << binaryFile << std::endl;

    //Create kernel
    auto krnl = xrt::kernel(device, uuid, "rtl_kernel_wizard_0", xrt::kernel::cu_access_mode::exclusive);

    //Allocate Buffer
    std::cout << "Allocate Buffer in Global Memory\n";
    auto bo0 = xrt::bo(device, sizeof(uint32_t) * DATA_SIZE, krnl.group_id(1));

    // Map the contents of the buffer object into host memory
    auto bo0_map = bo0.map<uint32_t*>();
    std::fill(bo0_map, bo0_map + DATA_SIZE, 0);

    // Create the test data
    uint32_t bufReference[DATA_SIZE];
    for (int i = 0; i < DATA_SIZE; i++) {
        bo0_map[i] = i;
        bufReference[i] = i + 1;
    }

    // Synchronize buffer content with device side
    std::cout << "synchronize input buffer data to device global memory\n";
    bo0.sync(XCL_BO_SYNC_BO_TO_DEVICE);

    // Execution of the kernel
    uint32_t d_scalar00 = 0;
    // bo0_addr = bo0.address();
    // krnl.write_register(ADDR, data);
    // krnl.read_register(ADDR, data);

    std::cout << "Execution of the kernel\n";
    auto run = krnl(d_scalar00, bo0);
    run.wait();

    // Get the output;
    std::cout << "Get the output data from the device" << std::endl;
    bo0.sync(XCL_BO_SYNC_BO_FROM_DEVICE);

    // Validate our results
    if (std::memcmp(bo0_map, bufReference, DATA_SIZE))
        throw std::runtime_error("Value read back does not match reference");

    std::cout << "TEST PASSED\n";
    return 0;

}
