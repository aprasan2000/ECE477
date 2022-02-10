#include "pch.h"

using namespace System;
using namespace System::IO::Ports;

int main(array<System::String^>^ args)
{
    SerialPort port("COM3", 9600);
    port.Open();

    while(true)
    {
        String^ data_rx = port.ReadLine();
        Console::WriteLine(data_rx)
    }
}