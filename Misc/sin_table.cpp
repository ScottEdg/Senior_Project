/*
Written By: Scott Edgerly
Date: 10/19/2015
Description:  Simple code to generate a sin Look Up Table (LUT).
Note: This program only gets the positive values of a sine wave 0 degrees to 180 degrees. 
      The rest of the array is zeroed out unless otherwise specified.
*/

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <fstream>
#define PI 3.14159265359
using namespace std;

void sin90(int C, ofstream& fout); //This is the default case (i.e prints floating point values) from 0 to 90 degrees
void sin180(int C, ofstream& fout); //same as sine90 but prints an array from 0 to 180 degrees
void sin360(int C, ofstream& fout); //same as sine180 but prints 0's after 180.
void int_sin90(int C, int max); //print integer values based on a max integer value from 0 to 90
void int_sint180(int C, int max); //same as int_sin90 but prints values from 0 to 180
void int_sin360(int C, int max); //same but as int_sine180 but prints 0's after 180

int main(){
	ofstream fout;
	string array_type = "int"; //can be changed to what ever data type is needed
	string array_name = "sin_table"; //can be changed to any name following C/C++ syntax
	double step_size = 1; // in degrees
	int array_capacity = 360 * (1.0/step_size); //can be change to whatever the user needs.
	int max_value = 16800;  //this value is used as an integer reference (such as the max value of an Auto Reload Register)
	int mode[3] = {4, 2, 1}; //mode[0] is 0 to 90, mode[1] is 0 to 180, mode[2] is 0 to 180 with injected 0's
	int offset = 0;
	int div = mode[2];

	fout.open("sin_table.txt"); //file name can be changed but make sure .txt is present
	

	if(div == 4){
		offset = 1;
	}
	int n = (array_capacity / div) + offset;
	fout<<array_type<<" "<<array_name<<"["<<n<<"] = { "; //prints array type, name and capacity
	if(max_value == 1){ // if user enters 1 for max_value then do fixed point mode
		for(int i = 1; i<n + 1;i++){
			
			if(i < n && div != 1){
				fout<<sin((2*(i-1)*PI)/360)<<", ";
			}
			
			if(i == n && div != 1){
				fout<<sin((2*(i-1)*PI)/360);
			}
			
			if(i <= n/2 && div == 1){
				fout<<sin((2*(i-1)*PI)/360)<<", ";
			}
			
			if(i > n/2 && div == 1 ){
				fout<<0<<", ";
			}
			if(i % 10 == 0){
				fout<<endl;
			}
			
		}
	}
	else{
		for(int i = 1; i<n + 1;i++){
			
			if(i < n && div != 1){
				fout<<int(max_value*sin((2*(i-1)*PI)/360))<<", ";
			}
			
			if(i == n && div != 1){
				fout<<int(max_value*sin((2*(i-1)*PI)/360));
			}
			
			if(i <= n/2 && div == 1){
				fout<<int(max_value*sin((2*(i-1)*PI)/360))<<", ";
			}
			
			if(i > n/2 && div == 1 ){
				fout<<0<<", ";
			}
			if(i % 10 == 0){
				fout<<endl;
			}
			
		}
	}	

	fout<<"};";
	fout.close();
	return 0;
}



