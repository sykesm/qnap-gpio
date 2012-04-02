#include <stdio.h>
#include <stdlib.h>
#include <linux/input.h>
#include <fcntl.h>

int main(int argc, char **argv)
{
int fd;
if ((fd = open("/dev/input/event2", O_RDONLY)) < 0) {
perror("evdev open");
exit(1);
}

struct input_event ev;

while(1) {
read(fd, &ev, sizeof(struct input_event));
switch (ev.code) {
	case KEY_RESTART: 
		if (ev.value) printf("RESET KEY_UP\n"); 
		else printf("RESET KEY_DOWN\n");
		break;
	case KEY_ARCHIVE:
		if (ev.value) printf("USB COPY KEY_UP\n"); 
                else printf("USB COPY KEY_DOWN\n");
                break; 
}
}

return 0;
}
