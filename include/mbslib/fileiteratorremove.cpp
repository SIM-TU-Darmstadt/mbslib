#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <string>

using namespace std;

void changefilename(boost::filesystem::path p) {
    cout << "came" << endl;
    boost::filesystem::directory_iterator end_ptr;
    boost::filesystem::directory_iterator dir(p);

    for (; dir != end_ptr; dir++) {
        p = boost::filesystem::path(*dir);
        if (is_directory(p)) {
            changefilename(p);
        } else {
            if (p.extension() == ".cpp") {
                boost::filesystem::remove(p);
            }
        }
    }
}

int main() {
    boost::filesystem::directory_iterator end_ptr;
    boost::filesystem::directory_iterator dir("./");
    boost::filesystem::path * p = new boost::filesystem::path(*dir);

    string filename(dir->path().string());
    string pathName(dir->path().parent_path().string());
    string oldName(dir->path().filename().string());
    cout << filename << endl
         << pathName << endl
         << oldName << endl
         << dir->path().stem();
    changefilename("./");
    //cout << 10 << *start_ptr <<endl;//start_ptr->status();
    return 0;
}
