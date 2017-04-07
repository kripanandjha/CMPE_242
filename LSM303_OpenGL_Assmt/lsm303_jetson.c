#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pthread.h>
#include <fcntl.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/fs.h>
//#include <math.h>
#include <tgmath.h>

#include<GL/glut.h>
#include <GL/freeglut.h>
#include <GL/glfw.h>
#include "24cXX.h"
#define MS_WR 0x1E
#define I2C_SLAVE 0x0703
#define I2C_FORCE_SLAVE 0x0706
#include <unistd.h>

int lsm_read_byte(int fd, uint16_t addr)
{
	int r;
	usleep(10);
	ioctl(fd, BLKFLSBUF); // clear kernel read buffer
	__u8 buf =  addr & 0x0ff;
	r = i2c_smbus_write_byte(fd, buf);
	if(r < 0)
		fprintf(stderr, "Error i2c_write_1b: %s\n", strerror(errno));
	usleep(10);

	r = i2c_smbus_read_byte(fd);
	return r;
}

void *my_entry_function(void *param);

int g = 0;

void *my_entry_function(void *param)
{
	int r = 0;
	int fd = open("/dev/i2c-1", O_RDWR);
	if(fd <= 0)
	{
		fprintf(stderr, "Error i2c open: %s\n", strerror(errno));
		exit (1);
	}

	// set working device
	if( ( r = ioctl(fd, I2C_SLAVE, 0x1e)) < 0)
	{
		fprintf(stderr, "Error setting working device: %s\n", strerror(errno));
		exit (1);
	}
//label1:;


	uint8_t sub_add = 0x0;
	uint8_t data = 0x1c;
	r = i2c_smbus_write_byte_data(fd, sub_add, data);
	if(r < 0) {
		fprintf(stderr, "Error i2c_write to addr 0x0 : %s\n", strerror(errno));
		exit (1);
	}
	usleep(10);

//label1:;
	sub_add = 0x1;
	data = 0x20;
	r = i2c_smbus_write_byte_data(fd, sub_add, data);
	if(r < 0) {
		fprintf(stderr, "Error i2c_write to addr 0x01 : %s\n", strerror(errno));
		while(1);
		//exit (1);
	}
	usleep(10);

label1:;
	   sub_add = 0x2;
	   data = 0x0;
	   r = i2c_smbus_write_byte_data(fd, sub_add, data);
	   if(r < 0) {
		   fprintf(stderr, "Error i2c_write  to addr 0x02: %s\n", strerror(errno));
		   exit (1);
	   }
	   usleep(10);

	   float Pi = 3.14159;
	   uint8_t xh = lsm_read_byte(fd, 0x03);
	   fflush(stdout);
	   uint8_t xl = lsm_read_byte(fd, 0x04);
	   fflush(stdout);
	   //uint8_t zh = lsm_read_byte(fd, addr++);
	   //uint8_t zl = lsm_read_byte(fd, addr++);
	   uint8_t yh = lsm_read_byte(fd, 0x07);
	   fflush(stdout);
	   uint8_t yl = lsm_read_byte(fd, 0x08);
	   fflush(stdout);

	   int16_t X =(int16_t) (xl | ((int16_t)xh << 8)) ;
	   int16_t Y =(int16_t) (yl | ((int16_t)yh << 8)) ;
	   float x_axis = (float)X/1100.0F  * 100;// converting gauss into micro-tesla
	   float y_axis = (float)Y/1100.0F  * 100;
	   float heading = (atan2(y_axis,x_axis) * 180) / Pi;
	   printf(" xh %x xl %x yh %x yl %x\n", xh, xl, yh, yl);
	   //printf(" y axis %f x axis %f\n", y_axis, x_axis);
	   if(heading < 0)
		   heading = 360 + heading;

	   g = heading;
	   printf(" degree %f \n",heading);
	   usleep(1000);
	   goto label1;

}

GLuint  loadAndBufferImage(const char* filename)
{
	GLFWimage imageData;
	glfwReadImage(filename, &imageData, 0);

	GLuint textureBufferID;
	glGenTextures(1, &textureBufferID);
	glBindTexture(GL_TEXTURE_2D, textureBufferID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imageData.Width, imageData.Height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imageData.Data);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glfwFreeImage(&imageData);
	return textureBufferID;	
}
void drawSquare(){
    glBegin(GL_QUADS);
        glVertex2f(-0.5,-0.5);
        glVertex2f(-0.5,0.5);
        glVertex2f(0.5,0.5);
        glVertex2f(0.5,-0.5);
    glEnd();

}

void drawLines()
{	
	glBegin(GL_LINES);
        glColor3f(1,0,0);
        glVertex2f(0,0);
        glColor3f(0,1,0);
        glVertex2f(0,0.5);
    glEnd();
}

void drawdir()
{
        glColor3f(1,0,1);
	glRasterPos2f( 0, 0.6); // location to start printing text
	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, 'N'); // Print a character on the screen

	glRasterPos2f( 0, -0.6); // location to start printing text
	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, 'S'); // Print a character on the screen

	glRasterPos2f( -0.6, 0); // location to start printing text
	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, 'W'); // Print a character on the screen

	glRasterPos2f( 0.6, 0); // location to start printing text
	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, 'E'); // Print a character on the screen

}
void mydisplay()
{
    //glClear(GL_COLOR_BUFFER_BIT);
#if 1
	drawSquare();
	drawdir();
	glPushMatrix();
	glRotatef(g, 0,0,1);
	drawLines();
	glPopMatrix();

    glutSwapBuffers();
#endif
}

int main(int argc, char** argv)
{
	glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA);
    glutInitWindowSize(1024, 768);
    glutInitWindowPosition(100, 100);
	glutCreateWindow("lsm303 compass");
	//glutDisplayFunc(mydisplay);
    glutIdleFunc(mydisplay);


    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	pthread_t thread_1;

	pthread_create(&thread_1, NULL, my_entry_function, NULL );
	glutMainLoop();

	printf("control never comes here\n");

	return 0;
}
