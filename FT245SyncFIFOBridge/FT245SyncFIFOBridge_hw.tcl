# TCL File Generated by Component Editor 18.1
# Tue Sep 03 11:45:20 JST 2019
# DO NOT MODIFY


# 
# FT245SyncFIFOBridge "FT245 Style Synchronous FIFO to Avalon-ST Bridge" v1.0
#  2019.09.03.11:45:20
# 
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1

# 
# module FT245SyncFIFOBridge
# 
set_module_property DESCRIPTION ""
set_module_property NAME FT245SyncFIFOBridge
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property AUTHOR ""
set_module_property DISPLAY_NAME "FT245 Style Synchronous FIFO to Avalon-ST Bridge"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property VALIDATION_CALLBACK verify_parameters
set_module_property ELABORATION_CALLBACK change_interfaces

# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL FT245SyncFIFOBridge
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE true
add_fileset_file FT245SyncFIFOBridge.sv SYSTEM_VERILOG PATH FT245SyncFIFOBridge.sv TOP_LEVEL_FILE

add_fileset SIM_VERILOG SIM_VERILOG "" ""
set_fileset_property SIM_VERILOG TOP_LEVEL FT245SyncFIFOBridge
set_fileset_property SIM_VERILOG ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property SIM_VERILOG ENABLE_FILE_OVERWRITE_MODE true
add_fileset_file FT245SyncFIFOBridge.sv SYSTEM_VERILOG PATH FT245SyncFIFOBridge.sv


# 
# parameters
# 
add_parameter DATA_WIDTH INTEGER 8
set_parameter_property DATA_WIDTH DISPLAY_NAME "Data Width"
set_parameter_property DATA_WIDTH ALLOWED_RANGES {8 16 32}
set_parameter_property DATA_WIDTH UNITS bits
set_parameter_property DATA_WIDTH DISPLAY_HINT radio
set_parameter_property DATA_WIDTH HDL_PARAMETER true

add_parameter EMPTY_WIDTH INTEGER 0
set_parameter_property EMPTY_WIDTH DISPLAY_NAME "Empty Width of Avalon-ST"
set_parameter_property EMPTY_WIDTH UNITS bits
set_parameter_property EMPTY_WIDTH DERIVED true
set_parameter_property EMPTY_WIDTH HDL_PARAMETER true

add_parameter BE_WIDTH INTEGER 1
set_parameter_property BE_WIDTH DISPLAY_NAME "Byte Enable Width of FT245"
set_parameter_property BE_WIDTH UNITS bits
set_parameter_property BE_WIDTH DERIVED true
set_parameter_property BE_WIDTH HDL_PARAMETER true

add_parameter MIN_ARBITRATION_CYCLES INTEGER 8
set_parameter_property MIN_ARBITRATION_CYCLES DISPLAY_NAME "Minimum Arbitration Cycles"
set_parameter_property MIN_ARBITRATION_CYCLES DESCRIPTION "Minimum clock cycles to switch bus direction."
set_parameter_property MIN_ARBITRATION_CYCLES ALLOWED_RANGES 8:1024
set_parameter_property MIN_ARBITRATION_CYCLES UNITS cycles
set_parameter_property MIN_ARBITRATION_CYCLES AFFECTS_ELABORATION false
set_parameter_property MIN_ARBITRATION_CYCLES HDL_PARAMETER true

add_parameter MAX_ARBITRATION_CYCLES INTEGER 1024
set_parameter_property MAX_ARBITRATION_CYCLES DISPLAY_NAME "Maximum Arbitration Cycles"
set_parameter_property MAX_ARBITRATION_CYCLES DESCRIPTION "Maximum clock cycles to switch bus direction."
set_parameter_property MAX_ARBITRATION_CYCLES ALLOWED_RANGES 16:65535
set_parameter_property MAX_ARBITRATION_CYCLES UNITS cycles
set_parameter_property MAX_ARBITRATION_CYCLES AFFECTS_ELABORATION false
set_parameter_property MAX_ARBITRATION_CYCLES HDL_PARAMETER true

proc verify_parameters {} {
    set data_width [get_parameter_value DATA_WIDTH]
    if {$data_width == 8} {
        set_parameter_value EMPTY_WIDTH 1
        set_parameter_value BE_WIDTH 1
    } elseif {$data_width == 16} {
        set_parameter_value EMPTY_WIDTH 1
        set_parameter_value BE_WIDTH 2
    } elseif {$data_width == 32} {
        set_parameter_value EMPTY_WIDTH 2
        set_parameter_value BE_WIDTH 4
    } else {
        send_message error "Illegal value for DATA_WIDTH parameter."
    }
    
    set min_arb_cycels [get_parameter_value MIN_ARBITRATION_CYCLES]
    set max_arb_cycels [get_parameter_value MAX_ARBITRATION_CYCLES]
    if {$max_arb_cycels <= $min_arb_cycels} {
        send_message error "'Minimum Arbitration Cycles' must be lower than 'Maximum Arbitration Cycles'."
    }
}

proc change_interfaces {} {
    set data_width [get_parameter_value DATA_WIDTH]
    if {$data_width == 8} {
        set_port_property ft_be TERMINATION true
        set_port_property sink_endofpacket TERMINATION true
        set_port_property sink_startofpacket TERMINATION true
        set_port_property sink_empty TERMINATION true
        set_port_property source_empty TERMINATION true
    } else {
        set_port_property ft_be TERMINATION false
        set_port_property sink_endofpacket TERMINATION false
        set_port_property sink_startofpacket TERMINATION false
        set_port_property sink_empty TERMINATION false
        set_port_property source_empty TERMINATION false
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
set_interface_property reset associatedClock ""
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
set_interface_property sink readyLatency 1
set_interface_property sink ENABLED true
set_interface_property sink EXPORT_OF ""
set_interface_property sink PORT_NAME_MAP ""
set_interface_property sink CMSIS_SVD_VARIABLES ""
set_interface_property sink SVD_ADDRESS_GROUP ""

add_interface_port sink sink_data data Input DATA_WIDTH
add_interface_port sink sink_empty empty Input EMPTY_WIDTH
add_interface_port sink sink_endofpacket endofpacket Input 1
add_interface_port sink sink_ready ready Output 1
add_interface_port sink sink_startofpacket startofpacket Input 1
add_interface_port sink sink_valid valid Input 1


# 
# connection point conduit
# 
add_interface conduit conduit end
set_interface_property conduit associatedClock ""
set_interface_property conduit associatedReset ""
set_interface_property conduit ENABLED true
set_interface_property conduit EXPORT_OF ""
set_interface_property conduit PORT_NAME_MAP ""
set_interface_property conduit CMSIS_SVD_VARIABLES ""
set_interface_property conduit SVD_ADDRESS_GROUP ""

add_interface_port conduit ft_data data Bidir DATA_WIDTH
add_interface_port conduit ft_be byteenable Bidir BE_WIDTH
add_interface_port conduit ft_txe_n txe_n Input 1
add_interface_port conduit ft_rxf_n rxf_n Input 1
add_interface_port conduit ft_oe_n oe_n Output 1
add_interface_port conduit ft_rd_n rd_n Output 1
add_interface_port conduit ft_wr_n wr_n Output 1


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
set_interface_property source readyLatency 1
set_interface_property source ENABLED true
set_interface_property source EXPORT_OF ""
set_interface_property source PORT_NAME_MAP ""
set_interface_property source CMSIS_SVD_VARIABLES ""
set_interface_property source SVD_ADDRESS_GROUP ""

add_interface_port source source_data data Output DATA_WIDTH
add_interface_port source source_empty empty Output EMPTY_WIDTH
add_interface_port source source_endofpacket endofpacket Output 1
add_interface_port source source_startofpacket startofpacket Output 1
add_interface_port source source_ready ready Input 1
add_interface_port source source_valid valid Output 1
set_port_property source_empty TERMINATION_VALUE 0
