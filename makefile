src_c:= "src_c/"
all:
	gcc -g $(src_c)azamipng.c -o azamipng.bin
clean:
	rm *.bin *.png
