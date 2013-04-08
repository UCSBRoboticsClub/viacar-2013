#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H



template <typename T, int S>
class CircularBuffer {
public:
    CircularBuffer() {
        headIndex = 0;
    }
    T read(int index);
    T& operator[](int index);
    void write(T value, int index);
    void push(T value);
    void revert(int amount);

protected:
    T data[S];
    int headIndex;
    inline int getRealIndex(int i);
};



/* getRealIndex()
 *
 * Translates head-based indices into absolute indices.
 * Private function used by other members.
 */
template <typename T, int S>
inline int CircularBuffer<T, S>::getRealIndex(int index) {
    int realIndex = (headIndex - index) % S;
    if (realIndex < 0) realIndex += S;
    return realIndex;
}



/* read(int index)
 *
 * Returns the object stored at a given index.
 */
template <typename T, int S>
inline T CircularBuffer<T, S>::read(int index) {
    return data[getRealIndex(index)];
}



/* operator[int index]
 *
 * Returns a reference to the object stored at a given index.
 */
template <typename T, int S>
inline T& CircularBuffer<T, S>::operator[](int index) {
    return data[getRealIndex(index)];
}



/* write(T value, int index)
 *
 * Sets the value of the object stored at the given index.
 */
template <typename T, int S>
inline void CircularBuffer<T, S>::write(T value, int index) {
    data[getRealIndex(index)] = value;
}



/* push(T value)
 *
 * Pushes the given value to the head of the buffer.
 * Moves the head forward.
 */
template <typename T, int S>
inline void CircularBuffer<T, S>::push(T value) {
    ++headIndex %= S;
    data[headIndex] = value;
}



/* revert(int amount)
 *
 * Pushes the head back by a given amount.
 */
template <typename T, int S>
inline void CircularBuffer<T, S>::revert(int amount) {
    headIndex = headIndex >= amount ? headIndex - amount : headIndex - amount + S;
}



#endif // CIRCULARBUFFER_H