# arguments:
#   r4 - FIFO index bitmask
#   r5 - PWMSAR address
#   r6 - FIFO start physical address
#   r7 - channel context address
#   PDA - EPIT1_SR address
#   MDA - GPIO port address
#   MS - 0x00000000 (source and destination address frozen; start in read mode)
#   PS - 0x000c0400 (destination address frozen; 32-bit write size; start in write mode)
#   CA (r7, 18) - GPIO lock bitmask (see below)
#   scratch[5] (r7, 29) - FIFO "in" index (tail)
#   scratch[6] (r7, 30) - direction; 0x00000001=forward, 0xFFFFFFFF=backward
#   scratch[7] (r7, 31) - waypoint interrupt counter (see below)

# register usage:
#   r0 - temporary
#   r1 - internal flags (see below)
#   r2 - current pulse data byte/temporary
#   r3 - GPIO pin values to be written
#   scratch[0] (r7, 24) - X position in steps (signed)
#   scratch[1] (r7, 25) - Y position in steps (signed)
#   scratch[2] (r7, 26) - Z position in steps (signed)
#   scratch[3] (r7, 27) - number of bytes processed
#   scratch[4] (r7, 28) - FIFO "out" index (head)
#
# r0-r7 only need to be set once, when the script is loaded.
# scratch0-scratch4 hold the script's current state
# scratch5 must be updated when more data is enqueued
#
# If scratch7 > 0, it is decremented once per byte (regardless of whether data
# is being processed forward or backward.)
# When the value reaches 0, a host interrupt is sent, but data continues to
# be processed. Can be used to signal the host at an arbitrary point in the cut.
# (similar to a scanline interrupt on old game consoles)
#
# If scratch7 is 0, a host interrupt is only generated when the entire FIFO
# has been consumed.
#
# GPIO lock bitmask:
#   The inverse of this 32-bit field is ANDed with the value to be written to
#   the GPIO port before each step. To lock certain motors, set the bits
#   corresponding to their STEP pins.
#   bit 20 set - Y1_STEP always low (Y1 motor locked)
#   bit 21 set - Z_STEP always low (Z motor locked)
#   bit 22 set - X_STEP always low (X motor locked)
#   bit 29 set - Y2_STEP always low (Y2 motor locked)
#   Setting bits other than these is undefined behavior.
#
# r1 flags:
#   bit 0 - always set, can be tested for unconditional jumps
#   bit 1 - set if last byte processed was a power level
#           (prevents processing multiple power levels in a row)
#   bit 2 - set if another byte should be processed immediately
#           (instead of waiting for the next timer interrupt)

start:
  ldi r1, 1     # constant

  # initialize the GPIO output value
  ldi r3, 0

  # Do nothing the first time around.
  # Clear the timer interrupt and wait for another timer tick.
  # This ensures we perform exactly one iteration when the script starts.
  # (The very first time the script is loaded, execution starts when the channel
  # priority is set. If there is also a pending timer event, we could wind up
  # doing two ticks worth of work in one.)
  stf r1, 0xc8      # (PD) clear EPIT interrupt flag
  done 4


# Main loop iteration; executed every EPIT interrupt
fifo_consume_byte:
  stf r1, 0xc8      # (PD) clear EPIT interrupt flag

fifo_consume_byte_no_iflag_clear:
  # protect against null pointer deref; exit if buffer address is 0
  cmpeqi r6, 0
  bt alldone

  # if head == tail, FIFO is empty
  ld r0, (r7, 28)   # get head index
  and r0, r4        # apply bitmask
  ld r2, (r7, 29)   # get tail index
  and r2, r4        # apply bitmask
  cmpeq r0, r2
  bt alldone

  # read byte of pulse data from offset in r0
  add r0, r6
  stf r0, 0x10  # (MSA|FR) set source address, no autoincrement
  ldf r2, 0x09  # (MD|SZ8) read byte from source address

  # if high bit is set, it's a power level
  btsti r2, 7
  bt do_power

  # if running through data backwards, invert direction bits
  ld r0, (r7, 30) # get increment
  btsti r0, 31    # check if negative
  bf do_x_dir
  xori r2, 0x4a   # increment is negative, invert X_DIR/Y_DIR/Z_DIR
  bclri r2, 4     # always keep laser off when running backwards

  # set X direction bit
do_x_dir:
  bseti r3, 19  # set X_DIR bit (pin 19 in GPIO bank)
  btsti r2, 1   # X_DIR: bit 1 in pulse byte
  bf do_y_dir
  bclri r3, 19  # clear X_DIR bit if bit in pulse byte was set

  # set Y direction bits
  # (note: Y1 and Y2 need to spin in opposite directions)
do_y_dir:
  bclri r3, 28  # clear Y2_DIR bit (pin 28 in GPIO bank)
  bseti r3, 18  # set Y1_DIR bit (pin 18 in GPIO bank) because it needs to be the inverse of Y2_DIR
  btsti r2, 3   # Y_DIR: bit 3 in pulse byte
  bf do_z_dir
  bseti r3, 28  # set Y2_DIR bit if bit in pulse byte was set
  bclri r3, 18  # clear Y1_DIR bit if bit in pulse byte was set

  # set Z direction bit
do_z_dir:
  bclri r3, 17  # clear Z_DIR bit (pin 17 in GPIO bank)
  btsti r2, 6   # Z_DIR: bit 6 in pulse byte
  bf set_dir
  bseti r3, 17  # set Z_DIR bit if bit in pulse byte was set

  # write direction bits to GPIO register
set_dir:
  stf r3, 0x2b  # (MD|SZ32|FL) write to GPIO register

  # short delay before setting the step bits (min setup time is 650 ns)
  ldi r0, 13
  loop do_x_step, 0
  mov r1, r1
  mov r1, r1
  mov r1, r1

  # set X step bit (it's already cleared from the previous iteration)
do_x_step:
  btsti r2, 0   # X_STEP: bit 0 in pulse byte
  bf do_y_step
  bseti r3, 22  # X_STEP: pin 22 in GPIO bank

  # set Y step bits (they're already cleared from the previous iteration)
do_y_step:
  btsti r2, 2   # Y_STEP: bit 2 in pulse byte
  bf do_z_step
  bseti r3, 20  # Y1_STEP: pin 20 in GPIO bank
  bseti r3, 29  # Y2_STEP: pin 29 in GPIO bank

  # set Z step bit (it's already cleared from the previous iteration)
do_z_step:
  btsti r2, 5   # Z_STEP: bit 5 in pulse byte
  bf do_laser
  bseti r3, 21  # Z_STEP: pin 21 in GPIO bank

  # set laser bit
do_laser:
  bclri r3, 30  # clear LASER_ENABLE bit (pin 30 in GPIO bank)
  bclri r3, 31  # clear LASER_ON_HEAD bit (pin 31 in GPIO bank)
  btsti r2, 4   # LASER: bit 4 in pulse byte
  bf set_step_and_laser
  bseti r3, 30  # LASER_ENABLE: pin 30 in GPIO bank
  bseti r3, 31  # LASER_ON_HEAD: pin 31 in GPIO bank

  # write step and laser enable bits to GPIO register
set_step_and_laser:
  ld r0, (r7, 18) # apply motor lock
  andn r3, r0
  stf r3, 0x2b  # (MD|SZ32|FL) write to GPIO register

  # short delay to give the pulse some width (min pulse width is 1.9 microseconds)
  ldi r0, 50
  loop upd_x_pos, 0
  mov r1, r1
  mov r1, r1
  mov r1, r1

upd_x_pos:
  # update position counters
  btsti r2, 0     # X_STEP: bit 0 in pulse byte
  bf upd_y_pos    # if X_STEP not set, don't update x position
  ld r0, (r7, 24) # get x position
  btsti r2, 1     # X_DIR: bit 1 in pulse byte
  bf upd_x_pos2   # if X_DIR not set, add 1
  subi r0, 2      # otherwise subtract 1 (subtract 2, add 1)
upd_x_pos2:
  addi r0, 1
  st r0, (r7, 24) # save new x position

upd_y_pos:
  btsti r2, 2     # Y_STEP: bit 2 in pulse byte
  bf upd_z_pos    # if Y_STEP not set, don't update y position
  ld r0, (r7, 25) # get y position
  btsti r2, 3     # Y_DIR: bit 3 in pulse byte
  bt upd_y_pos2   # if Y_DIR set, add 1
  subi r0, 2      # otherwise subtract 1 (subtract 2, add 1)
upd_y_pos2:
  addi r0, 1
  st r0, (r7, 25) # save new y position

upd_z_pos:
  btsti r2, 5     # Z_STEP: bit 5 in pulse byte
  bf upd_pos_done # if Z_STEP not set, don't update z position
  ld r0, (r7, 26) # get z position
  btsti r2, 6     # Z_DIR: bit 6 in pulse byte
  bt upd_z_pos2   # if Z_DIR set, add 1
  subi r0, 2      # otherwise subtract 1 (subtract 2, add 1)
upd_z_pos2:
  addi r0, 1
  st r0, (r7, 26) # save new z position

upd_pos_done:
  # clear the "just processed a power level" flag
  bclri r1, 1

clear_step_bits:
  # clear step bits
  bclri r3, 22  # X_STEP: pin 22 in GPIO bank
  bclri r3, 20  # Y1_STEP: pin 20 in GPIO bank
  bclri r3, 29  # Y2_STEP: pin 29 in GPIO bank
  bclri r3, 21  # Z_STEP: pin 21 in GPIO bank

  # write cleared step bits to GPIO register
  stf r3, 0x2b  # (MD|SZ32|FL) write to GPIO register

endbyte:
  # increment/decrement the byte counter
  ld r2, (r7, 30) # get the increment value (+1 or -1)
  ld r0, (r7, 27)
  add r0, r2
  st r0, (r7, 27)

  # increment/decrement head index (add is sufficient, don't need bitmask)
  ld r0, (r7, 28)
  add r0, r2      # +1 or -1
  st r0, (r7, 28)

  # decrement waypoint counter if > 0
  ld r0, (r7, 31)
  cmpeqi r0, 0
  bt endbyte_done   # ignore if scratch7 == 0
  subi r0, 1        # always decrement, T flag set if result is zero
  st r0, (r7, 31)
  bf endbyte_done
  done 3            # waypoint reached, trigger a host interrupt

endbyte_done:
  # if bit 2 in r1 was set, process another byte immmediately
  btsti r1, 2   # updates T flag
  bclri r1, 2   # leaves T flag unchanged
  bt fifo_consume_byte_no_iflag_clear

  # done with this byte, clear event flag and wait for next event
  done 4
  btsti r1, 0   # always true
  bt fifo_consume_byte



alldone:
  done 3        # trigger a host interrupt
  done 4

  # when script is restarted, jump back to beginning
  btsti r1, 0   # always true
  bt start



# sets the PWM duty cycle based on the lower 7 bits or r2,
# then processes another byte
# we ensure that we process at most one power level change per timer interrupt;
# if the previously-processed byte was a power level change, bit 1 of r1 will be set
do_power:
  btsti r1, 1   # don't process two power level changes in a row
  bt endbyte    # if so, wait for the next interrupt

  # if running backwards, ignore all power changes
  ld r0, (r7, 30) # get increment
  btsti r0, 31    # check if negative
  bt power_done

  # need to get PWMSAR address into PDA
  ldf r0, 0xd0  # (PDA) save previous value (EPIT_SR address)
  stf r5, 0xd3  # (PDA|SZ32|F) set PWMx_PWMSAR as destination address
  andi r2, 0x7f # mask high bit of r2
  stf r2, 0xc8  # (PD) write new PWM duty cycle value
  stf r0, 0xd3  # (PDA|SZ32|F) restore EPIT_SR address
  bseti r1, 1   # set bit 1 indicating we just processed a power level change

power_done:
  # immediately process another byte
  # (set bit 2 in r1 before jumping so we don't wait for another interrupt)
  bseti r1, 2
  btsti r1, 0   # always true
  bt endbyte    # unconditional jump
