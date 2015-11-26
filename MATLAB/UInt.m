function UInt = UInt(t, U)
%UINT Zeitintegration
%Autor: Niklas Fauth / 2015-11-25
%Beschreibung: Diese Funktion approximiert die Integration
%              mit der Trapezregel.
%              Sie akzeptiert einen Zeitvektor t und
%              einen Vektor mit den abzuleitenden Werten
%              Als Rueckgabewert liefert diese Funktion
%              einen Vektor mit den Integralwerten.

if (length(t) ~= length(U)) %Check if vectors are of the same lenght
    vectorLength = min([length(t) length(U)]); %if not, use the shorter one
    if (vectorLength == length(t))used
        vectorName = 'time';        
    else
        vectorName = 'input';
    end
    
    %display a warning including the name of the used vector
    warning('The input vectors of UInt do not have the same length. The %s vector will be used.', vectorName);
else
    vectorLength = length(t);
end

% Initialization of the returned vector.
UInt = zeros(1, vectorLength);

for i = 1 : vectorLength % Calculate the integration.
      
    % Check for last value in vector.
    if(i == vectorLength)
        break;
        
    elseif(i == 1)
        Usum = 0;
        
    else
        % Difference between two time values.
        dt = t(i + 1) - t(i);

        % Sum of two input values.
        Usum = ((U(i) + U(i + 1)) / 2 * dt) + Usum;
        UInt(i) = Usum;
    end
end
end