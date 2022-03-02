// Copyright (c) Team 2393, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util;

import java.util.Arrays;

/** Circular buffer
 * 
 *  Aka 'Ring Buffer', an array of items with fixed length == capacity.
 *  As items are added, the array fills as one would expect with any
 *  array until reaching the capacity:
 *    items[0] = first item
 *    items[1] = second item
 *    items[2] = 3rd item
 *    ...
 *  As more items are added, older items are replaced.
 *  Note that this is done without copying/shifting existing array items.
 *  For small buffers it would be OK to simply move all array items 'down':
 *    items[0] = second item
 *    items[1] = 3rd item
 *    ...
 *  
 *  But this is inefficient for larger arrays, since _all_ array elements
 *  need to be moved whenever a new value is added.
 * 
 *  Instead, the array index is cleverly wrapped around the array length,
 *  treating what's a linear array in memory as a 'ring' or 'circular' buffer
 *  without need to move any items around.
 * 
 *  Also note that the circular buffer doesn't really care what type of
 *  item it holds. Could be String, Integer, or any class.
 *  So it's defined as a generic for a type 'T', whatever that might be.
 *  The only requirement for 'T' is that it can be printed,
 *  i.e. defines 'toString()', to get meaningful output from 'dump()'.
 * 
 *  @param T Circular buffer item type
 */
public class CircularBuffer<T>
{
  /** Buffer items
   * 
   *  Length of this array is the maximum ring buffer capacity
   */
  private final T[] items;

  /** Valid size of buffer, i.e. number of items */
  private int size = 0;

  /** Index where the next item will be written */
  private int next = 0;

  /** Create circular buffer
   *  @param capacity Number of items the buffer can hold before it replaces older items
   */
  @SuppressWarnings("unchecked")
  public CircularBuffer(final int capacity)
  {
    items = (T[]) new Object[capacity];
  }

  /** @param item Item to add  */
  public void add(final T item)
  {
    // 'next' is index where new item is written
    // (or old item is overwritten)
    items[next] = item;

    // Advance 'next', wrapping around at ring buffer length
    next = (next + 1) % items.length;

    // Keep track of how many items we added from 0 to items.length-1.
    // Once we reach capacity, we'll replace older items.
    if (size < items.length)
      ++size;
  }

  /** @return Return number of valid items, 0 ...capacity */
  public int size()
  {
    return size;
  }

  /** @param index Index 0 .. size()-1 of item to get
   *  @return That item
   *  @throws IllegalArgumentException for invalid index
   */
  public T get(final int index)
  {
    if (index < 0)
      throw new IllegalArgumentException("What's wrong with you??");
    if (index >= size)
      throw new IllegalArgumentException("Invalid index "  + index + " for circular buffer of size " + size);

    // 'next' is where the next item will be written, i.e. one after the most recently added item.
    // Basically, 'next - size' is the index of the first item,
    // but beware that 'next' might be 0 as we wrap around,
    // so add items.length to make sure it's a positive number.
    final int first = next - size + items.length;
    // 'first + index' is the requested item.
    // Finally, don't forget to wrap around at items.length.
    final int actual = (first + index) % items.length;
    return items[actual];
  }

  /** Remove all elements */
  public void clear()
  {
    size = next = 0;

    // Fundamentally, it all "works" without actually removing the old items.
    // With size == 0, the buffer will appear empty (size() will return 0).  
    // As we add() new items, that would replace the old items that are still in the buffer.
    // But it's a good idea to explicitly remove the old items:
    Arrays.fill(items, null);

    // That's for two reasons:
    // 1) If we don't add() new items, the old items will still be held
    //    in memory. In case they're large, that would waste memory.
    // 2) In case we have a bug somewhere that would expose old items.
  }    

  /** Print all items */
  public void dump()
  {
    System.out.println("CircularBuffer:");
    if (size() <= 0)
      System.out.println("- empty -");
    else
      for (int i=0; i<size(); ++i)
        System.out.println(i + ": " + get(i));
  }

  public static void main(String[] args)
  {
    final CircularBuffer<String> buffer = new CircularBuffer<>(2);
    buffer.dump();

    buffer.add("One");
    buffer.dump();

    buffer.add("Two");
    buffer.dump();

    buffer.add("Three");
    buffer.dump();

    buffer.add("Four");
    buffer.dump();

    buffer.add("Five");
    buffer.dump();

    buffer.clear();
    buffer.add("Six (after clear)");
    buffer.dump();
  }
}