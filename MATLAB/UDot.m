function UDot = UDot(t, U)

%UDOT Zeitableitung
%Autor: Niklas Fauth / 2015-11-25
%Beschreibung: Diese Funktion approximiert die Ableitung
%              mit dem Vorwaertsdifferenzenquotienten.
%              Sie akzeptiert einen Zeitvektor t und
%              einen Vektor mit den abzuleitenden Werten
%              Als Rueckgabewert liefert diese Funktion
%              einen Vektor mit den Ableitungswerten.

if (length(t) ~= length(U)) %Check if vectors are of the same lenght
    vectorLength = min([length(t) length(U)]); %if not, use the shorter one
    if (vectorLength == length(t))     
        vectorName = 'time';        
    else
        vectorName = 'input';
    end
    
    %display a warning including the name of the used vector
    warning('The input vectors of UDot do not have the same length. The %s vector will be used.', vectorName);
else
    vectorLength = length(t);
end

% Initialization of the returned vector.
UDot = zeros(1, vectorLength);

for i = 1 : vectorLength  % Calculate the derivation.
    
    % Check for last value in vector.
    if (i == vectorLength)
        UDot(i-1) = UDot(i);
    else
        % Difference between two time values.
        dt = t(i+1) - t(i);

        % Difference between two input values.
        dU = U(i+1) - U(i);
        
        % calculate the actual derivation.
        UDot(i) = dU/dt;
    end
end
end

