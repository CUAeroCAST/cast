function str = argjoin(arg)
 arg = string(arg);
 while numel(arg) > 1
  arg = join(arg);
 end
 str = arg;
end
