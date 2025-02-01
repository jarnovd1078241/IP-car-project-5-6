#ifndef INC_MAIN_LOOP_HPP_
#define INC_MAIN_LOOP_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/* Function for main to call to prevent overwriting when generating code from CubeMX. */
void main_loop_c();

#ifdef __cplusplus
}
#endif

#endif /* INC_MAIN_LOOP_HPP_ */
