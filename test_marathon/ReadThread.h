#ifndef READTHREAD_H
#define READTHREAD_H

#include <QTimer>
#include <QThread>
#include <iostream>
#include "../lib/chai/src/chai.h"

class ReadThread : public QThread {
Q_OBJECT
public:
    int CHANEL_NUMBER;
    ReadThread();

signals:
    void data_signal(canmsg_t *rx_buffer);


protected:
    void run();
};

#endif  /* READTHREAD_H */