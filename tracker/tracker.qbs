import qbs 1.0


Product {
    type: "application"
    Depends { name: "cpp" }
    files: [
        "board.h",
        "main.cpp",
        "queue.h",
        "stm32f0.ld",
        "stm32f4.ld",
        "uart.cpp",
        "uart.h",
    ]
    property string vendor: "STM"
    property string model: "STM32F4DISCOVERY"
    property string toolchain: "GCC_ARM"
    property string cortex: "M4"
    cpp.includePaths: [ 'C:\\src\\eigen\\', 'C:\\src\\flyflow\\tracker\\CMSIS\\Include' ]
    cpp.commonCompilerFlags: [
        //"-mthumb","-mcpu=cortex-m0",
        "-mthumb","-mcpu=cortex-m4",
        "-mfloat-abi=hard","-mfpu=fpv4-sp-d16",
        "-fdata-sections","-ffunction-sections",
        "-fno-inline","-std=c++14","-flto"]

    cpp.linkerFlags:[
        "-flto","-mthumb","-mcpu=cortex-m4",
        "-mfloat-abi=hard","-mfpu=fpv4-sp-d16",
        "--specs=nano.specs","-Wl,--start-group",
        "-Wl,--gc-sections",
        "-T", path + "/stm32f4.ld",
        "-lnosys","-lgcc","-lc"]
    Properties {
        condition: qbs.buildVariant === "debug"
        cpp.defines: outer.concat(["DEBUG=1"])
    }
    Group {     // Properties for the produced executable
        fileTagsFilter: "application"
        qbs.install: true
    }
}
