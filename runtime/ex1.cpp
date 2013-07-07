/**
 * @file ex1.cpp
 * @author Can Erdogan
 * @date June 06, 2013
 * @brief This file is the first example of runtime errors, specifically focusing on pointers.
 * The idea is that we will create a dictionary reading sorted words from a file and search 
 * words the user requests. We will not use stl classes that can make this job easier but
 * instead play with pointers. 
 */

#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <assert.h>
#include <stdlib.h>

using namespace std;

// Let's assume there will be 39 words and each word is at most 8 letters long
#define NUM_WORDS 38
#define MAX_NUM_LETTERS 10

/// Reads the dictionary from a local file
void readDictionary (char*** dictionary) {

	// Open the file
	ifstream file ("words.txt", ifstream::in);
	assert(file.is_open() && "Could not open the file!");

	// Read the lines one by one - assume each word is on a line
	char line [1024];
	size_t wordCounter = 0;
	while(file.getline(line, 1024) && wordCounter < NUM_WORDS) {
		char* temp = strcpy((*dictionary)[wordCounter++], line);
	}
}

/// Prints the contents of the dictionary
void printDictionary (char** dictionary) {
	printf("Dictionary: \n");
	for(size_t i = 0; i < NUM_WORDS; i++)
		printf("\t%s\n", dictionary[i]);
}

/// Searches for a word in the dictionary with binary search. If the word is in 
/// the dictionary, returns true and sets the input index pointer to the right value. 
/// Otherwise, returns false.
bool findWord (char** dictionary, const char* word, size_t start_idx, size_t end_idx, size_t* goal) {

	// Check if the word is before the start index or after the index
	if(strcmp(dictionary[start_idx], word) > 0) return false;
	else if(strcmp(dictionary[end_idx], word) < 0) return false;

	// Check if we recursed so deep that the start and the end index are the same. If so, the word has to be
	// this index for this recursion to return successfully.
	if(start_idx == end_idx) {

		// Set the output if successful, otherwise return false
		if(strcmp(dictionary[start_idx], word) == 0) {
			*goal = start_idx;
			return true;
		}
		else return false;
	}

	// ===========================================================================
	// Now that we know it is somewhere between, we divide the job into two parts: 
  // [start_idx, middle] and [middle+1, end_idx]. If either is successful, return true.

	// Try the first part
	size_t middle_idx = (start_idx + end_idx) / 2;
	// size_t *middle_idx = new size_t;
	// *middle_idx = (start_idx + end_idx) / 2;
	bool successPart1 = findWord(dictionary, word, start_idx, middle_idx, goal);
	if(successPart1) return true;

	// Try the second part, its success is this call's success now.
	bool successPart2 = findWord(dictionary, word, middle_idx+1, end_idx, goal);
	return successPart2;
}

/// The main thread
int main () {

	// Allocate memory for the dictionary
	char** dictionary = new char* [NUM_WORDS];
	for(size_t word_index = 0; word_index < NUM_WORDS; word_index++) {
		dictionary[word_index] = new char [MAX_NUM_LETTERS];
	}

	// Populate the entries of the dictionary
	readDictionary(&dictionary);
	printDictionary(dictionary);

	// Prompt the user for words to search for
	char word [16];
	while(true) {

		// Get the word
		printf("\nEnter a word: ");
		scanf("%s", &word[0]);
		printf("entered: '%s'\n", word);

		// Search for the word
		size_t* index = (size_t*)malloc(sizeof(size_t));
		bool result = findWord(dictionary, word, 0, NUM_WORDS-1, index);
		printf("result: %d, index: %lu\n", result, *(index) + 1);
  }
}
