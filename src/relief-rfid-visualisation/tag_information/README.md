`tag_info.txt` should include entries
EPCAAA,typeAAA
EPCAAB,typeAAB
...
EPCZZZ,typeZZZ
unknown,unknown

where EPCXXX is a tag's EPC, typeXXX its type, and at the end **there should
be a line with `unknown`,`unknown`** so that a tag that is not included in
`tag_info.txt` is still represented in the map, but with an unknown image.
