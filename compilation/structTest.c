#include <stdio.h>

struct B{
	int a;
};

int main(){
	struct B b1;  
	b1.a = 10;

	struct B b2;  
	b2.a = 20;

	struct B *b3 = new struct B; 
	b3->a = 30;

	printf("%d\n",b1.a);
	printf("%d\n",b2.a);
	printf("%d\n",b3->a);

	return 1;
}