#ifndef HOS_CHAR_H
#define HOS_CHAR_H

int char2hex(char);
char hex2char(int);
int ishex(char);
int stringcompare(char*, char*, int, int);
int search_int(char*, int);
void long2ascii( signed long, char* );

#endif