#include <iostream>

using namespace std;

int multiple(int n1, int n2){
    if((n1>=0 && n2>=0) || (n1<=0 && n2<=0)){
        return (n1*n2);
    }else{
        return -(n1*n2);
    }
        
}

int main(int argc, char **argv){
    int N, a, b,res;
    
    cout << "Type N:" << endl;
    cin >> N;
    
    for(int i=0;i<N;i++){
        cout << "Type a and b: " << endl;
        cin >> a >> b;
        
        res = multiple(a,b);
        
        cout << "Result: " << res << endl;
    
    }
    
    return 0;
}

