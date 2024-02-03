import gdb

class LoadImageCommand(gdb.Command):
    """Load an image file."""

    def __init__(self):
        # The first argument is the new command name 'loadimage'
        super(LoadImageCommand, self).__init__("loadimage", gdb.COMMAND_USER)

    def invoke(self, arg, from_tty):
        # Your command implementation here
        # 'arg' will contain any arguments passed to 'loadimage'
        
        # Extracting arguments
        breakpoint_location =  "frame_handler.c:375"
        memory_start = "buffer_context.img_head_ptr"
        memory_end = "buffer_context.img_tail_ptr"

        dump_path = "./imgdump"
        # Setting a breakpoint
        gdb.execute(f"break {breakpoint_location}")


        # - loop begin -
        while True:
            # Continuing execution
            gdb.execute("continue")

            id =  gdb.parse_and_eval("args->image_storage_struct->id")
            timestamp =  gdb.parse_and_eval("args->image_storage_struct->timestamp")
            # Dumping memory
            gdb.execute(f"dump memory {dump_path}/{id}_{timestamp}.jpeg {memory_start} {memory_end}")

            print(f"Memory dumped from {memory_start} to {memory_end} into {dump_path}/{id}_{timestamp}.jpeg")
        # - loop end -


# Creating an instance of the command to register it with GDB
LoadImageCommand()

