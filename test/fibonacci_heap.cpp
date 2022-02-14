#include "FibonacciHeap.h"
#include <iostream>

using namespace std;

struct Vertex {
  size_t ind_;
  float f_;
  void set(size_t ind, float f) {
    ind_ = ind;
    f_ = f;
  };
  bool operator>(const Vertex &rhs) const { return f_ > rhs.f_; };
  friend std::ostream &operator<<(std::ostream &os, const Vertex &vx);
};

std::ostream &operator<<(std::ostream &os, const Vertex &vx) {
  os << "(" << vx.ind_ << ", " << vx.f_ << ")";
  return os;
}

/*
 * main() for testing constructor, getMinimum(), display(), removeMinimum(),
 * decreaseKey(), isEmpty()
 */
int main(int argc, char **argv) {
  FibonacciHeap<Vertex> fh;

  Vertex vx1;
  vx1.set(1, 2.0f);
  fh.insert(vx1);

  Vertex vx2;
  vx2.set(2, 1.0f);
  fh.insert(vx2);

  fh.display();

  /*FibonacciHeap<int> fh;

  fh.insert(23);
  fh.insert(7);
  fh.insert(21);
  fh.insert(3);
  fh.insert(17);
  fh.insert(24);
  fh.insert(18);
  fh.insert(52);
  fh.insert(38);
  fh.insert(30);
  fh.insert(26);
  fh.insert(46);
  node<int> *n = fh.insert(39);
  node<int> *m = fh.insert(41);
  fh.insert(35);

  cout << "Heap Minimum: " << fh.getMinimum() << endl;
  cout << "The Heap is: " << endl;

  fh.display();
  cout << "Heap Minimum Extracted: " << fh.removeMinimum() << endl;
  fh.display();

  cout << "de: " << n->getValue() << " para: 5" << endl;
  fh.decreaseKey(n, 5);

  cout << "Heap Minimum: " << fh.getMinimum() << endl;
  fh.display();

  cout << "de: " << m->getValue() << " para: 2" << endl;
  fh.decreaseKey(m, 2);

  while (!fh.isEmpty()) {
    cout << "Heap Minimum Extracted: " << fh.removeMinimum() << endl;
    fh.display();
  }*/

  return 0;
}
