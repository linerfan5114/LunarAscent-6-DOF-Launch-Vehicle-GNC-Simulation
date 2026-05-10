LunarAscent/
├── src/
│   ├── vehicle.cpp              ← مدل موشک (جرم، آیرودینامیک، مرکز ثقل)
│   ├── propulsion.cpp          ← مدل موتور (تراست، Isp، gimbal)
│   ├── environment.cpp         ← مدل محیط (جو، گرانش، باد)
│   ├── guidance.cpp            ← قانون هدایت (میخواهیم کجا بریم)
│   ├── navigation.cpp          ← ناوبری (الان کجاییم)
│   ├── control.cpp             ← کنترل (موتور رو کج کنیم)
│   └── main.cpp                ← شبیه‌ساز اصلی
├── include/
│   ├── vehicle.hpp
│   ├── propulsion.hpp
│   ├── environment.hpp
│   ├── guidance.hpp
│   ├── navigation.hpp
│   └── control.hpp
├── sim/
│   └── plot_trajectory.py
├── Makefile
├── README.md
└── LICENSE