/*
Здесь объявляем собственный макрос rassert. Он имеет следующий вид:
rassert(cond, str);
Здесь cond - условие, которое обязано выполняться (это мы проверяем) - если оно нарушится, программа аварийно завершится,
str - строка, которую вывести, если условие нарушено.
*/


#pragma once

//#define DEBUG  // объявляем переменную DEBUG, если хотим, чтобы rassert работал

#ifdef DEBUG
    #define rassert(condition, info) if (!(condition)) { throw std::runtime_error("Assertion failed!\n" "Add info: " info); }
#else
    #define rassert(condition, info) (void) 1;  // если отключаем rassert, о заменяет его на строку (void) 1, которая
                                                // фактически ничего не делает (можно ещё заменять на просто ; - но если
                                                // rassert был внутри if, то после такой замены if станет пустым и будет ошибка
                                                // -Wempty-body)
#endif
