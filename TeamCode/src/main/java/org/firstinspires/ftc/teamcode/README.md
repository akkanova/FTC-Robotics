# Directory Structure
A program is like a house: just as walls give a house structure, separation of concerns 
gives a program structure. Programmer discipline is required to create imaginary walls 
where no connections are allowed. We separate the code into a `generic` area and a `specific`
area, allowing the loosely coupled universal code to be reused in other projects.

```
all_purpose
 └── Highly universal code. Broadly useful in a wide variety of 
     applications. E.g. Data-Structure
     
common
 └── Somewhat universal code. Potentially useful in many applications,
     but designed with the codebase more in mind.
     
human_operated
 └── TeleOp Human Control Scripts
 
self_driving
 └── Also known as Autonomous Scripts
 
tests
 └── Hardware and Software Debug Code
```

As it goes down, the code becomes more specific and tightly coupled.
