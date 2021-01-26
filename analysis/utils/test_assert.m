function test_assert(truth, file, line, arg1, arg2)
 % custom assertion for test cases
 global failCount;
 global assertCount;
 assertCount = assertCount + 1;
 if nargin > 4
  try
    if truth
     assert(all(arg1 == arg2, 'all'));
    else
     assert(any(arg1 ~= arg2, 'all'));
    end
  catch
   if truth
    fprintf(file,  '    Line %i: Assertion Failed: %s == %s\n', line, argjoin(arg1), argjoin(arg2));
   else
    fprintf(file,  '    Line %i: Assertion Failed: %s != %s\n', line, argjoin(arg1), argjoin(arg2));
   end
   failCount = failCount + 1;
  end
 else
  try
   if truth
    assert(arg1);
   else
    assert(~arg1);
   end
  catch
   fprintf(file, '    Line %i: Assertion Failed: %s != %s\n', line, argjoin(arg1), string(truth));
   failCount = failCount + 1;
  end
 end
end

