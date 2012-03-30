#ifndef MEDIATOR_HPP_INCLUDED
#define MEDIATOR_HPP_INCLUDED

/* original source: http://ideone.com/8VVEa */

#define ItemLess(a,b)  ((a)<(b))
#define ItemMean(a,b)  (((a)+(b))/2)
#define MinCt(pm) (((pm)->itemCount-1)/2) //count of items in minheap
#define MaxCt(pm) (((pm)->itemCount)/2)   //count of items in maxheap

template <typename Item, int N>
class Mediator
{

   public:
      Mediator()
      {
         int nItems = N;

         itemCount = idx = 0;
         while (nItems--)  //set up initial heap fill pattern: median,max,min,max,...
         {
            pos[nItems]= ((nItems+1)/2) * ((nItems&1)?-1:1);
            heapd[N/2+pos[nItems]]=nItems;
         }
      }

      //Inserts item, maintains median in O(lg nItems)
      void Insert(Item newValue)
      {
         int isNew = (itemCount < N);
         int insertAt = pos[idx];
         Item curValue = data[idx];

         data[idx] = newValue;
         idx = (idx + 1) % N;
         itemCount += isNew;
         if (insertAt > 0)         //new item is in minHeap
         {
            if (!isNew && ItemLess(curValue,newValue))
            {
               minSortDown(insertAt * 2);
            }
            else if (minSortUp(insertAt))
            {
               maxSortDown(-1);
            }
         }
         else if (insertAt < 0)   //new item is in maxheap
         {
            if (!isNew && ItemLess(newValue,curValue))
            {
               maxSortDown(insertAt * 2);
            }
            else if (maxSortUp(insertAt))
            {
               minSortDown(1);
            }
         }
         else            //new item is at median
         {
            if (maxCt())
            {
                maxSortDown(-1);
            }
            if (minCt())
            {
               minSortDown( 1);
            }
         }
      }

      //returns median item (or average of 2 when item count is even)
      Item Median()
      {
         Item v = data[heapd[N/2]];
         if ((itemCount & 1) == 0)
         {
            v = ItemMean(v,data[heapd[N/2+-1]]);
         }
         return v;
      }

   private:
      Item data[N];  //circular queue of values
      int  pos[N];   //index into `heap` for each value
      int  heapd[N];  //max/median/min heap holding indexes into `data`.
      int  idx;   //position in circular queue
      int  itemCount;    //count of items in queue

      int minCt()
      {
         return (itemCount-1) / 2;
      }

      int maxCt()
      {
         return itemCount / 2;
      }

      int mmless(int i, int j)
      {
         return ItemLess(data[heapd[N/2+i]],data[heapd[N/2+j]]);
      }

      //swaps items i&j in heap, maintains indexes
      int mmexchange(int i, int j)
      {
         int t = heapd[N/2+i];
         heapd[N/2+i]=heapd[N/2+j];
         heapd[N/2+j]=t;
         pos[heapd[N/2+i]]=i;
         pos[heapd[N/2+j]]=j;
         return 1;
      }

      int mmCmpExch(int i, int j)
      {
         return (mmless(i,j) && mmexchange(i,j));
      }

      //maintains minheap property for all items below i/2.
      void minSortDown(int i)
      {
         for (; i <= minCt(); i*=2)
         {  if (i>1 && i < minCt() && mmless(i+1, i)) { ++i; }
            if (!mmCmpExch(i,i/2)) { break; }
         }
      }

      //maintains maxheap property for all items below i/2. (negative indexes)
      void maxSortDown(int i)
      {
         for (; i >= -maxCt(); i*=2)
         {  if (i<-1 && i > -maxCt() && mmless(i, i-1)) { --i; }
            if (!mmCmpExch(i/2,i)) { break; }
         }
      }

      //maintains minheap property for all items above i, including median
      //returns true if median changed
      int minSortUp(int i)
      {
         while (i>0 && mmCmpExch(i,i/2)) i/=2;
         return (i==0);
      }

      //maintains maxheap property for all items above i, including median
      //returns true if median changed
      int maxSortUp(int i)
      {
         while (i<0 && mmCmpExch(i/2,i))  i/=2;
         return (i==0);
      }
};


#endif // MEDIATOR_HPP_INCLUDED
