#ifndef LYN_COMPONENTS_H
#define LYN_COMPONENTS_H
#include <lyn_info.h>
#include <lyn_log.h>
#include <lyn_meta.h>
#include <lyn_link.h>
#include <lyn_joint.h>
class components
{
public:
    components();
    std::string name;
    lyn_joint joint;
    lyn_link link;
    std::map<std::string, components> Components;
};
#endif // LYN_COMPONENTS_H
