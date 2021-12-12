QT -= gui core

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += RV_ARCH=32
DEFINES += USE_SIMPLE_UART
DEFINES += NDEBUG
# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    ../src/core/core.c \
    ../src/core/csr/csr.c \
    ../src/core/mmu/mmu.c \
    ../src/core/pmp/pmp.c \
    ../src/core/trap/trap.c \
    ../src/helpers/fifo.c \
    ../src/helpers/file_helper.c \
    ../src/main.c \
    ../src/peripherals/clint/clint.c \
    ../src/peripherals/plic/plic.c \
    ../src/peripherals/uart/simple_uart.c \
    ../src/peripherals/uart/uart_8250.c \
    ../src/soc/riscv_example_soc.c

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    ../src/core/core.h \
    ../src/core/csr/csr.h \
    ../src/core/mmu/mmu.h \
    ../src/core/pmp/pmp.h \
    ../src/core/riscv_config.h \
    ../src/core/riscv_helper.h \
    ../src/core/riscv_instr.h \
    ../src/core/riscv_types.h \
    ../src/core/riscv_xlen_specifics.h \
    ../src/core/trap/trap.h \
    ../src/helpers/fifo.h \
    ../src/helpers/file_helper.h \
    ../src/peripherals/clint/clint.h \
    ../src/peripherals/plic/plic.h \
    ../src/peripherals/uart/simple_uart.h \
    ../src/peripherals/uart/uart_8250.h \
    ../src/soc/riscv_example_soc.h


INCLUDEPATH += /home/user/riscv_em/riscv_em/src/helpers/ \
/home/user/riscv_em/riscv_em/src/peripherals/ \
/home/user/riscv_em/riscv_em/src/peripherals/plic/ \
/home/user/riscv_em/riscv_em/src/peripherals/uart/ \
/home/user/riscv_em/riscv_em/src/peripherals/clint/ \
/home/user/riscv_em/riscv_em/src/soc/ \
/home/user/riscv_em/riscv_em/src/core/ \
/home/user/riscv_em/riscv_em/src/core/csr/ \
/home/user/riscv_em/riscv_em/src/core/mmu/ \
/home/user/riscv_em/riscv_em/src/core/pmp/ \
/home/user/riscv_em/riscv_em/src/core/trap/ \
/home/user/riscv_em/riscv_em/src/peripherals/




