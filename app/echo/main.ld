ENTRY(main)
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K

}

SECTIONS
{
	.text :
	{
		KEEP(*(.isr_vector))
 		*(.text)
 		*(.text.*)
		*(.rodata)
		*(.rodata.*)
		_sidata = .;
	} >FLASH

	/* Initialized data will initially be loaded in FLASH at the end of the .text section. */
    .data : AT (_sidata) 
    {
		_sdata = .;
		*(.data)		/* Initialized data */
		_edata = .;
	} >RAM


	.ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  .ARM.exidx : 
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH


	.bss : {
		_sbss = .;
		*(.bss)         /* Zero-filled run time allocate data memory */
		_ebss = .;
	} >RAM

  . = ALIGN(4);
  _end = .;

    _estack = ORIGIN(RAM) + LENGTH(RAM);
 }  