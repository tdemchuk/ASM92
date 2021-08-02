# This program will repeatedly increment a value at some memory location by
# a constant of 0x05. The program halts when an arithemetic overflow is detected 
# (when the V flag goes high). The value of the sum is constantly displays on the output.

# Note: a bug exists where labels must include at minimum one valid hexadecimal
#   character (0-9, A-F) to jump to. This is an unintended byproduct of the 
#   instruction identification and validation process. Hence why 'loop1' is 
#   used instead of 'loop'.


# 0xC0 is output buffer
# 0xC4 is input buffer
# 0xC8 is PSW

init:
    mov $50, 0x12   # sum is stored at 0x50. init to 0x12
    mov $C0, $50    # update display

loop1:
    cmp 1           # clear PSW flags to avoid unintended add w/ carry
    add $50, 05     # increment sum by 0x05
    mov $C0, $50    # update display

    mov $51, $C8    # load PSW to 0x51
    and $51, 4      # isolate V flag
    cmp $51, 4      # zero set if V flag set
    brz exit        # exit if overflow occurs

    jmp loop1       # iterate

exit:
    hlt             # terminate program