:- module(knowledge_util,[string_to_list/2]).


string_to_list(StringValue, List):-
  atomic_list_concat(StringList,' ', StringValue),
  to_double_list(StringList, [], List).

to_double_list([], TempList, DoubleList):-
  append([], TempList, DoubleList).

to_double_list([H|T], TempList, DoubleList):-
  atom_number(H, DoubleValue),
  append(TempList, [DoubleValue], NewTempList),
  to_double_list(T, NewTempList, DoubleList).