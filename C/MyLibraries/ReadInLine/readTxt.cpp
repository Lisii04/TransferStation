#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

class Menu
{
    private:
        string text;
        
    public:

        Menu()
        {
            
        }

        Menu(string text)
        {
            this->text = text;
        }

        void changeText(string text)
        {
            this->text = text;
        }

        string getText()
        {
            return text;
        }


};

typedef struct Info
{
    string No;
    string FartherNo;
    string Text;
}Info;



vector<Info> read_from_file(char const *fileName)
{
    char Texts[256];
    char numbers[256]={};
    char content[256]={};
    string No,FartherNo,Text;
    int lineCount = 0;

    vector<Info> Infos;

    ifstream in;

    in.open(fileName);
    if(!in.is_open())
    {
        cout << "file not exit" << endl;
    }
    while(in.getline(Texts,256))
    {
        int i = 0,j = 0;
        Info* info = new Info;
        Infos.push_back(*info);

        while(Texts[i] != '\0')
        {
            if(Texts[i] == '[')
            {
                i++;
                while (Texts[i] != ']')
                {
                    numbers[i-1] = Texts[i];
                    i++;
                }
                No = numbers;
                Infos[lineCount].No = No;
                
                if(No.size() > 1)
                {
                    No.pop_back();
                    No.pop_back();
                    FartherNo = No;
                }else{
                    FartherNo = No;
                }
            }
            i++;
            content[j] = Texts[i]; 
            j++;
        }
        Text = content;

       
        Infos[lineCount].FartherNo = FartherNo;
        Infos[lineCount].Text = Text;
        lineCount ++;
    }

    return Infos;
}

int main(int argc, char** argv)
{
    char fileName[] = "./123.txt";

    vector<Info> Infos = read_from_file(fileName);
    

    for (auto i = 0; i < 10; i++)
    {
        cout << Infos[i].No << endl;
        cout << Infos[i].FartherNo << endl;
        cout << Infos[i].Text << endl;
        cout << "-----------" << endl;
    }
    

    return 0;
}

