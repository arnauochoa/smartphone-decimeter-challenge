function structC = appendStruct(structA, structB, dim)
if nargin<1
    selfTest();
    return;
end

if isempty(structA)
    structC = structB;
    return
end

fieldsA = fieldnames(structA);
assert(all(ismember(fieldsA, fieldnames(structB))), ...
    'MATLAB:invalidStruct', 'Fields differ between structures');

for i = 1:length(fieldsA)
    switch dim
        case 1
            structC.(fieldsA{i}) = [structA.(fieldsA{i}); structB.(fieldsA{i})]; 
        case 2
            structC.(fieldsA{i}) = [structA.(fieldsA{i}) structB.(fieldsA{i})];
        otherwise
            error('Handling of more than 2 array dimensions not implemented')
    end
end

end

function selfTest()

structA.a = 1:3;
structB.b = 5:6;
passed = false;
try
    appendStruct(structA, structB, 2);
catch e
    if e.identifier == 'MATLAB:invalidStruct'
        passed = true;
    else
        rethrow(e);
    end
end

structA.a = 1:3;
structA.b = 'abc';
structA.c = [1 1; 2 2];
structB.a = 4:6;
structB.b = 'def';
structB.c = [3 3; 4 4];

structC = appendStruct(structA, structB, 2);
passed = passed && all(structC.a == 1:6);
passed = passed && all(structC.b == 'abcdef');
passed = passed && all(structC.c == [1 1 3 3; 2 2 4 4], 'all');

structC = appendStruct(structA, structB, 1);
passed = passed && all(structC.a == [1:3;4:6], 'all');
passed = passed && all(structC.b == ['abc'; 'def'], 'all');
passed = passed && all(structC.c == [1 1; 2 2; 3 3; 4 4], 'all');

assert(passed, 'Function self test error');
disp([mfilename '>> Function self test(s) passed']);
end