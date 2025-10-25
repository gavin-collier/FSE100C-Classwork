% Gavin, Collier, 1225104777

userNum = input("Please enter a number: ")

for x = 1:userNum
    if mod(x,3) == 0 || mod(x,7) == 0
        if mod(x,21) == 0
            fprintf("%d\n", x)
        end
    else
        fprintf("%d\n", x)
    end
end