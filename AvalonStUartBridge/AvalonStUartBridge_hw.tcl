# TCL File Generated by Component Editor 18.1
# Sat Sep 21 15:40:41 JST 2019
# DO NOT MODIFY


# 
# AvalonStUartBridge "Avalon-ST UART Bridge" v1.0
#  2019.09.21.15:40:41
# 
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module AvalonStUartBridge
# 
set_module_property DESCRIPTION ""
set_module_property NAME AvalonStUartBridge
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property AUTHOR ""
set_module_property DISPLAY_NAME "Avalon-ST UART Bridge"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property VALIDATION_CALLBACK verify_parameters


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL AvalonStUartBridge
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE true
add_fileset_file AvalonStUartBridge.sv VERILOG PATH AvalonStUartBridge.sv TOP_LEVEL_FILE

add_fileset SIM_VERILOG SIM_VERILOG "" ""
set_fileset_property SIM_VERILOG TOP_LEVEL AvalonStUartBridge
set_fileset_property SIM_VERILOG ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property SIM_VERILOG ENABLE_FILE_OVERWRITE_MODE true
add_fileset_file AvalonStUartBridge.sv VERILOG PATH AvalonStUartBridge.sv


# 
# parameters
# 

add_parameter FULL_DUPLEX BOOLEAN 0
set_parameter_property FULL_DUPLEX DISPLAY_NAME "Full duplex"
set_parameter_property FULL_DUPLEX HDL_PARAMETER true
set_parameter_property FULL_DUPLEX AFFECTS_ELABORATION false

add_parameter INVERT_TXD_RXD BOOLEAN 0
set_parameter_property INVERT_TXD_RXD DISPLAY_NAME "Invert TXD and RXD"
set_parameter_property INVERT_TXD_RXD HDL_PARAMETER true
set_parameter_property INVERT_TXD_RXD AFFECTS_ELABORATION false

add_parameter INVERT_OE BOOLEAN 0
set_parameter_property INVERT_OE DISPLAY_NAME "Invert OE"
set_parameter_property INVERT_OE HDL_PARAMETER true
set_parameter_property INVERT_OE AFFECTS_ELABORATION false

add_parameter BAUD_RATE INTEGER 9600
set_parameter_property BAUD_RATE DISPLAY_NAME "Baud rate"
set_parameter_property BAUD_RATE ALLOWED_RANGES 1:2147483647
set_parameter_property BAUD_RATE AFFECTS_ELABORATION false

add_parameter CLOCK_RATE INTEGER 0
set_parameter_property CLOCK_RATE DISPLAY_NAME "Clock frequency"
set_parameter_property CLOCK_RATE SYSTEM_INFO {CLOCK_RATE clk}
set_parameter_property CLOCK_RATE AFFECTS_ELABORATION false
set_parameter_property CLOCK_RATE VISIBLE false

add_parameter PRESCALER INTEGER 0
set_parameter_property PRESCALER DISPLAY_NAME "Prescaler"
set_parameter_property PRESCALER DERIVED true
set_parameter_property PRESCALER HDL_PARAMETER true
set_parameter_property PRESCALER AFFECTS_ELABORATION false

add_parameter BAUD_RATE_ERROR FLOAT 0.0
set_parameter_property BAUD_RATE_ERROR DISPLAY_NAME "Baud rate error (%)"
set_parameter_property BAUD_RATE_ERROR DERIVED true
set_parameter_property BAUD_RATE_ERROR AFFECTS_ELABORATION false

proc verify_parameters {} {
    set baud_rate [get_parameter_value BAUD_RATE]
    set clock_rate [get_parameter_value CLOCK_RATE]
    set prescaler [expr $clock_rate / (8 * $baud_rate)]
    if {$clock_rate <= 0} {
        send_message error "Clock frequency is undetermined or zero."
    }
    if {$prescaler <= 0} {
        send_message error "Prescaler value is zero."
    }
    set_parameter_value PRESCALER $prescaler
    if {(0 < $clock_rate) && (0 < $prescaler)} {
        set actual_baud_rate [expr $clock_rate / (8 * $prescaler)]
        set error [expr 100.0 * ($actual_baud_rate - $baud_rate) / $baud_rate]
        set_parameter_value BAUD_RATE_ERROR $error
    } else {
        set_parameter_value BAUD_RATE_ERROR 100.0
    }
}


# 
# display items
# 


# 
# connection point clk
# 
add_interface clk clock end
set_interface_property clk clockRate 0
set_interface_property clk ENABLED true
set_interface_property clk EXPORT_OF ""
set_interface_property clk PORT_NAME_MAP ""
set_interface_property clk CMSIS_SVD_VARIABLES ""
set_interface_property clk SVD_ADDRESS_GROUP ""

add_interface_port clk clk clk Input 1


# 
# connection point reset
# 
add_interface reset reset end
set_interface_property reset synchronousEdges NONE
set_interface_property reset ENABLED true
set_interface_property reset EXPORT_OF ""
set_interface_property reset PORT_NAME_MAP ""
set_interface_property reset CMSIS_SVD_VARIABLES ""
set_interface_property reset SVD_ADDRESS_GROUP ""

add_interface_port reset reset reset Input 1


# 
# connection point sink
# 
add_interface sink avalon_streaming end
set_interface_property sink associatedClock clk
set_interface_property sink associatedReset reset
set_interface_property sink dataBitsPerSymbol 8
set_interface_property sink errorDescriptor ""
set_interface_property sink firstSymbolInHighOrderBits true
set_interface_property sink maxChannel 0
set_interface_property sink readyLatency 0
set_interface_property sink ENABLED true
set_interface_property sink EXPORT_OF ""
set_interface_property sink PORT_NAME_MAP ""
set_interface_property sink CMSIS_SVD_VARIABLES ""
set_interface_property sink SVD_ADDRESS_GROUP ""

add_interface_port sink sink_ready ready Output 1
add_interface_port sink sink_valid valid Input 1
add_interface_port sink sink_data data Input 8
add_interface_port sink sink_startofpacket startofpacket Input 1
add_interface_port sink sink_endofpacket endofpacket Input 1


# 
# connection point source
# 
add_interface source avalon_streaming start
set_interface_property source associatedClock clk
set_interface_property source associatedReset reset
set_interface_property source dataBitsPerSymbol 8
set_interface_property source errorDescriptor ""
set_interface_property source firstSymbolInHighOrderBits true
set_interface_property source maxChannel 0
set_interface_property source readyLatency 0
set_interface_property source ENABLED true
set_interface_property source EXPORT_OF ""
set_interface_property source PORT_NAME_MAP ""
set_interface_property source CMSIS_SVD_VARIABLES ""
set_interface_property source SVD_ADDRESS_GROUP ""

add_interface_port source source_ready ready Input 1
add_interface_port source source_valid valid Output 1
add_interface_port source source_data data Output 8
add_interface_port source source_startofpacket startofpacket Output 1
add_interface_port source source_endofpacket endofpacket Output 1
add_interface_port source source_error error Output 1


# 
# connection point uart
# 
add_interface uart conduit end
set_interface_property uart associatedClock clk
set_interface_property uart associatedReset reset
set_interface_property uart ENABLED true
set_interface_property uart EXPORT_OF ""
set_interface_property uart PORT_NAME_MAP ""
set_interface_property uart CMSIS_SVD_VARIABLES ""
set_interface_property uart SVD_ADDRESS_GROUP ""

add_interface_port uart uart_txd txd Output 1
add_interface_port uart uart_rxd rxd Input 1
add_interface_port uart uart_oe oe Output 1
