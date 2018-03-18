start:
  ldi r1, 1
  ldi r3, 0
  stf r1, 0xc8
  done 4
fifo_consume_byte:
  stf r1, 0xc8
fifo_consume_byte_no_iflag_clear:
  cmpeqi r6, 0
  bt alldone
  ld r0, (r7, 28)
  and r0, r4
  ld r2, (r7, 29)
  and r2, r4
  cmpeq r0, r2
  bt alldone
  add r0, r6
  stf r0, 0x10
  ldf r2, 0x09
  btsti r2, 7
  bt do_power
  ld r0, (r7, 30)
  btsti r0, 31
  bf do_x_dir
  xori r2, 0x4a
  bclri r2, 4
do_x_dir:
  bseti r3, 19
  btsti r2, 1
  bf do_y_dir
  bclri r3, 19
do_y_dir:
  bclri r3, 28
  bseti r3, 18
  btsti r2, 3
  bf do_z_dir
  bseti r3, 28
  bclri r3, 18
do_z_dir:
  bclri r3, 17
  btsti r2, 6
  bf set_dir
  bseti r3, 17
set_dir:
  stf r3, 0x2b
  ldi r0, 13
  loop do_x_step, 0
  mov r1, r1
  mov r1, r1
  mov r1, r1
do_x_step:
  btsti r2, 0
  bf do_y_step
  bseti r3, 22
do_y_step:
  btsti r2, 2
  bf do_z_step
  bseti r3, 20
  bseti r3, 29
do_z_step:
  btsti r2, 5
  bf do_laser
  bseti r3, 21
do_laser:
  bclri r3, 30
  bclri r3, 31
  btsti r2, 4
  bf set_step_and_laser
  bseti r3, 30
  bseti r3, 31
set_step_and_laser:
  ld r0, (r7, 18)
  andn r3, r0
  stf r3, 0x2b
  ldi r0, 50
  loop upd_x_pos, 0
  mov r1, r1
  mov r1, r1
  mov r1, r1
upd_x_pos:
  btsti r2, 0
  bf upd_y_pos
  ld r0, (r7, 24)
  btsti r2, 1
  bf upd_x_pos2
  subi r0, 2
upd_x_pos2:
  addi r0, 1
  st r0, (r7, 24)
upd_y_pos:
  btsti r2, 2
  bf upd_z_pos
  ld r0, (r7, 25)
  btsti r2, 3
  bt upd_y_pos2
  subi r0, 2
upd_y_pos2:
  addi r0, 1
  st r0, (r7, 25)
upd_z_pos:
  btsti r2, 5
  bf upd_pos_done
  ld r0, (r7, 26)
  btsti r2, 6
  bt upd_z_pos2
  subi r0, 2
upd_z_pos2:
  addi r0, 1
  st r0, (r7, 26)
upd_pos_done:
  bclri r1, 1
clear_step_bits:
  bclri r3, 22
  bclri r3, 20
  bclri r3, 29
  bclri r3, 21
  stf r3, 0x2b
endbyte:
  ld r2, (r7, 30)
  ld r0, (r7, 27)
  add r0, r2
  st r0, (r7, 27)
  ld r0, (r7, 28)
  add r0, r2
  st r0, (r7, 28)
  ld r0, (r7, 31)
  cmpeqi r0, 0
  bt endbyte_done
  subi r0, 1
  st r0, (r7, 31)
  bf endbyte_done
  done 3
endbyte_done:
  btsti r1, 2
  bclri r1, 2
  bt fifo_consume_byte_no_iflag_clear
  done 4
  btsti r1, 0
  bt fifo_consume_byte
alldone:
  done 3
  done 4
  btsti r1, 0
  bt start
do_power:
  btsti r1, 1
  bt endbyte
  ld r0, (r7, 30)
  btsti r0, 31
  bt power_done
  ldf r0, 0xd0
  stf r5, 0xd3
  andi r2, 0x7f
  stf r2, 0xc8
  stf r0, 0xd3
  bseti r1, 1
power_done:
  bseti r1, 2
  btsti r1, 0
  bt endbyte
