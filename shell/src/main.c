/* 
 * mcp4728 example
 * https://www.microchip.com/en-us/product/mcp4728
 * 
 * Copyright 2024 Aaron Mothar & Co Pty Ltd
 */ 

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#define NUM_OF_REG      5
uint8_t reg[NUM_OF_REG];
static int shell_subcmd_configure_handler(const struct shell *sh, size_t argc, char **argv);
static int shell_subcmd_set_handler(const struct shell *sh, size_t argc, char **argv);
static int shell_subcmd_read_handler(const struct shell *sh, size_t argc, char **argv);

// Shell configuration
SHELL_STATIC_SUBCMD_SET_CREATE
(sub_cmd,
	SHELL_CMD(configure, NULL, "configure", shell_subcmd_configure_handler),
        SHELL_CMD(set, NULL, "set <register 0 - 5> <0-255>", shell_subcmd_set_handler),
        SHELL_CMD(read, NULL, "read <register 0 - 5 >", shell_subcmd_read_handler),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(shell, &sub_cmd, "Shell", NULL);


int main(void)
{
        printk("Zephyr shell example\r\n");
        // Initialise all registers.
        for (int i = 0 ; i<NUM_OF_REG;i++)
        {
                reg[i] = 0;
        }

        while(1)
        {
                k_msleep(5);
                //printf(".");
        }
        return 0;
}

static int shell_subcmd_configure_handler(const struct shell *sh, size_t argc, char **argv)
{
        shell_print(sh,"Configure cmd received - BUT COMMAND NOT IMPLEMENTED YET\n");
        return 0;
}

static int shell_subcmd_set_handler(const struct shell *sh, size_t argc, char **argv)
{
        int reg_index = atoi(argv[1]); 
        int value = atoi(argv[2]); 

        if((reg_index >= NUM_OF_REG) || (reg_index < 0))
        {
                shell_print(sh,"Incorrect register number. Accepted values 0 - 5.");
                return -1;
        }

        if((value < 0) || (value > 255))
        {
                shell_print(sh,"Value %d, is out of bounds. Accepted values 0 - 5.", value);
                return -2;  
        }

        reg[reg_index] = value;
        return 0;

}

static int shell_subcmd_read_handler(const struct shell *sh, size_t argc, char **argv)
{
        int reg_index = atoi(argv[1]); 

        if((reg_index >= NUM_OF_REG) || (reg < 0))
        {
                shell_print(sh,"Incorrect register number. Accepted values 0 - 5.");
                return -1;
        }
        else
        {
                shell_print(sh,"%d", reg[reg_index]);
                return 0;
        }

}