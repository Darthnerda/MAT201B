#include <iostream>
#include <string>
using namespace std;

string reverser(string word); // declare reverser function prototype

int main() {
  while (true) {
    printf("Type a sentence (then hit return): ");
    string line;
    getline(cin, line);
    if (!cin.good()) {
      printf("Done\n");
      return 0;
    }
    string word;
    string out;
    for (int i = 0; i < line.length(); i++){
      if((int)line[i] != 32) { // unless its a space
        word += line[i]; // add it to this 'word'
      }
      else { // if it is a space, then
        out += reverser(word) + " "; // concatenate the reversed word to the output and add a space
        word =  ""; // and clear the word variable for the next part of the sentence
      }
    }
    // we must add the last reversed word, since there is no space to trigger it in the for loop, hence the '+ reverser(word)' below
    cout << out + reverser(word) + "\n"; // When the whole line is done, output the collected reversed words to console
  }
}

string reverser(string word) {
  string out;
  for (int j = word.length() - 1; j >= 0; j--) {
    out += word[j];
  }
  return out;
}