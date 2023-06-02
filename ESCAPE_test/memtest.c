
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <stdbool.h>
#include <sys/mman.h>

#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", \
		__LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

bool test_status = false; 
int main(int argc, char **argv) {
	int fd;
	void *map_base, *virt_addr; 
	unsigned long read_result, writeval;
	unsigned long  target;
	unsigned long size;
	int i = 0;
	int access_type = 'w';
	int test_type = 'r';

	if(argc < 4) {
		fprintf(stderr, "\nUsage:\t%s { address } {size} [ type] [test] [ data ] ]\n"
				"\taddress : memory address to act upon\n"
				"\tsize	   : memory size to act upon\n"
				"\ttype    : access operation type : [b]yte, [h]alfword, [w]ord\n"
				"\ttest    : test type : [r] read, [w] write, [c]read-compare\n"
				"\tdata    : data to be written\n\n",
				argv[0]);
		exit(1);
	}
	target = strtoul(argv[1], 0, 0);
	size = strtoul(argv[2], 0, 0);

	access_type = tolower(argv[3][0]);
	test_type = tolower(argv[4][0]);


	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) FATAL;
	//printf("/dev/mem opened.\n"); 
	fflush(stdout);

	/* Map one page */
	map_base = mmap(0, size , PROT_READ | PROT_WRITE, MAP_SHARED, fd, target & ~MAP_MASK);
	if(map_base == (void *) -1) FATAL;
	//printf("Memory mapped at address %p.\n", map_base); 
	fflush(stdout);
	virt_addr = map_base + (target & MAP_MASK);

	while(i < size ) {
		if (argc < 6 && test_type == 'r') {
			switch(access_type) {
				case 'b':
					read_result = *((unsigned char *)(virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr + i), read_result);
					i= i + sizeof(unsigned char);
					break;
				case 'h':
					read_result = *((unsigned short *)(virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr + i), read_result);
					i= i + sizeof(unsigned short);
					break;
				case 'w':
					read_result = *((unsigned long *)(virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr + i), read_result);
					i= i + sizeof(unsigned long);
					break;
				default:
					fprintf(stderr, "Illegal data type '%c'.\n", access_type);
					exit(2);
			}
			fflush(stdout);
		}
		if (argc > 5 && test_type == 'c') {
			writeval = strtoul(argv[5], 0, 0);
			switch(access_type) {
				case 'b':
					read_result = *((unsigned char *)(virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr +i), read_result);
					i= i + sizeof(unsigned char);
					break;
				case 'h':
					read_result = *((unsigned short *)(virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr +i), read_result);
					i= i + sizeof(unsigned short);
					break;
				case 'w':
					read_result = *((unsigned long *)(virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr +i), read_result);
					i= i + sizeof(unsigned long);
					break;
				default:
					fprintf(stderr, "Illegal data type '%c'.\n", access_type);
					exit(2);
			}
			if(read_result != writeval){
				printf("memory test read compare failed read_result =0x%lX\n",read_result);
				test_status = true;
				//break;
			}
			fflush(stdout);
		}
		if(argc > 5 && test_type == 'w') {
			writeval = strtoul(argv[5], 0, 0);
			switch(access_type) {
				case 'b':
					read_result = *((unsigned char *) (virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr +i), read_result);
					*((unsigned char *) (virt_addr + i)) = writeval;
					read_result = *((unsigned char *) (virt_addr + i));
					i= i + sizeof(unsigned char);
					break;
				case 'h':
					read_result = *((unsigned short *) (virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr +i), read_result);
					*((unsigned short *) (virt_addr + i)) = writeval;
					read_result = *((unsigned short *) (virt_addr + i));
					i= i + sizeof(unsigned short);
					break;
				case 'w':
					read_result = *((unsigned long *) (virt_addr + i));
					printf("Value at address 0x%lX (%p): 0x%lX\n", (target + i), (virt_addr +i), read_result);
					*((unsigned long *) (virt_addr + i)) = writeval;
					read_result = *((unsigned long *) (virt_addr + i));
					i= i + sizeof(unsigned long);
					break;
				default:
					fprintf(stderr, "Illegal data type '%c'.\n", access_type);
					exit(2);
			}
			if(read_result != writeval){
				printf("memory test write compare failed read_result =0x%lX\n",read_result);
				test_status = true;
				//break;
			}
			printf("Written 0x%lX; readback 0x%lX\n", writeval, read_result);
			fflush(stdout);
		}
	}
	if (test_status)
		printf("TEST FAILED\n");
	else
		printf("TEST PASSED\n");


	if(munmap(map_base, MAP_SIZE) == -1) FATAL;
	close(fd);
	return 0;
}


