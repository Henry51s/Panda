#pragma once

#include "Scanner.hpp"

class SScanner : public Scanner {

    private:

    Bank* bank;

    public:

    SScanner(Bank* bank_) : bank(bank_) {}

    void setup() override;
    void update() override;


};