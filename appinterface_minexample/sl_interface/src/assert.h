// strongly reduced version of assert.h from ROS

#define ROS_ASSERT(cond) \
   do { \
     if (!(cond)) { \
       std::cout << "ASSERTION FAILED" << std::endl; \
       abort(); \
     } \
   } while (false)

