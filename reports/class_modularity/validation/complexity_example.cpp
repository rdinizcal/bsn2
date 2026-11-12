class ComplexityExample {
public:
    int straight() { return 1; }
    int branch(bool value) {
        if (value) { return 1; }
        return 0;
    }
};
