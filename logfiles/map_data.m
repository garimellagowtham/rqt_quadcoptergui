function output = map_data(input, input_lowerbound, input_upperbound, output_lowerbound, output_upperbound)
      if input > input_upperbound
        output = output_upperbound;
      elseif input < input_lowerbound 
        output = output_lowerbound;
      else
        output = (output_lowerbound + ((input-input_lowerbound)*(output_upperbound-output_lowerbound))/(input_upperbound-input_lowerbound));
      end
end