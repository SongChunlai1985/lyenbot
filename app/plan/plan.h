#ifndef LYN_PLAN_H
#define LYN_PLAN_H
#include <avoiding.h>
#include <reconstruction.h>
// #include <face_recognition.h>
// #include <object_detection.h>

class plan
{
private:

public:
    plan();
    int run(int argc, char *argv[]);
    int work(lyn_info &info);

    avoiding Avoiding;
    reconstruction Reconstruction;
//     face_recognition Face_recognition;
//    object_detection Object_detection;

};
#endif // LYN_PLAN_H
