#include<map>
#include<iostream>
#include <string>
#include <string>
#include <vector>
#include <map>
#include<list>
//#include<queue>

using namespace std;
class MyResult
{
public:
    MyResult(float f,float h,string s)
        :f(f),h(h),s(s)
    {
        father = NULL;
    }
    float getF(){return f;}
    float getH(){return h;}
    string getS() const {return s;}
    void setF(float a){f = a;}
    MyResult * father;
private:
    float f,h;
    string s;

};
//struct MyCompare
//{
//    //f小在前面，h小在前面
//    bool operator()( MyResult * a, MyResult * b) const
//    {
//        return a->getF()>b->getF();
//    }
//};
struct MyCompare {
  bool operator()( float k1,  float k2) {
      return k1<k2;
  }
};

//void myPrintQueue(priority_queue<MyResult *,std::vector<MyResult *>,MyCompare> m_result){
//    while(!m_result.empty())
//    {
//        std::cout<<m_result.top()->getS()<<std::endl;
//        m_result.pop();
//    }
//}

int main(void)
{
multimap<float,MyResult *,MyCompare> open_queue;
list<MyResult *> global_path;
    MyResult r1(10,1,"r1"),r2(10,20,"r2"),r3(20,0,"r3"),r4(20,3,"r4");
    MyResult * p1 =&r1;
    MyResult * p2 =&r2;
    MyResult * p3 =&r3;
    MyResult * p4 =&r4;
    p2->father = p1;
    p3->father = p1;
    p4->father = p3;
    global_path.push_front(p4);

    MyResult * i = global_path.front();
    cout<<i->getS()<<endl;
    while(i->father != NULL){

        global_path.push_front(i->father);
        i = global_path.front();
         cout<<i->getS()<<endl;
    }
    cout<<global_path.size()<<endl;
//    MyResult * p = &r3;
//open_queue.insert(make_pair(r1.getF(),&r1));
//open_queue.insert(make_pair(r4.getF(),&r4));
//open_queue.insert(make_pair(r3.getF(),p));
//open_queue.insert(make_pair(r2.getF(),&r2));
//multimap<float,MyResult *,MyCompare>::iterator it = open_queue.begin();
// MyResult r5(-1,1,"r5");
// open_queue.insert(make_pair(r5.getF(),&r5));
// cout<<(it->second)->getS()<<endl;
//  it = open_queue.begin();
// while(open_queue.size() !=0){
//     cout<<(it->second)->getS()<<endl;
//     open_queue.erase(it++);
// }



//    std::priority_queue<MyResult *,std::vector<MyResult *>,MyCompare> m_result;
//    MyResult r1(10,1,"r1"),r2(10,20,"r2"),r3(20,0,"r3"),r4(20,3,"r4");
//    m_result.push(&r2);
//    m_result.push(&r4);
//    m_result.push(&r1);
//    MyResult * p3 = &r3;
//        MyResult r5(-1,1,"r5");
//    //    m_result.push(&r5);
//    m_result.push(p3);
//    bool run = true;
//    while(m_result.size() != 0)
//    {
//        MyResult * temp = m_result.top();
//        std::cout<<temp->getS()<<std::endl;
//        if(run){
//            m_result.push(&r5);
//            run =false;
//        }
//        m_result.pop();//delete r5
//    }
//    m_result.push(&r2);
//    m_result.push(&r4);
//    m_result.push(&r1);
//    m_result.push(p3);
//    p3->setF(0);
//    MyResult r5(-1,1,"r5");
//    m_result.push(&r5);
//    m_result.pop();
//    myPrintQueue(m_result);
//    cout<<"size "<<m_result.size()<<endl;


 return 0;
}

