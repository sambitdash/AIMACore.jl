# AIMACore

[![Build Status](https://travis-ci.org/sambitdash/AIMACore.jl.svg?branch=master)](https://travis-ci.org/sambitdash/AIMACore.jl)
[![Coverage Status](https://coveralls.io/repos/sambitdash/AIMACore.jl/badge.svg?branch=master&service=github)](https://coveralls.io/github/sambitdash/AIMACore.jl?branch=master)
[![codecov.io](http://codecov.io/github/sambitdash/AIMACore.jl/coverage.svg?branch=master)](http://codecov.io/github/sambitdash/AIMACore.jl?branch=master)
[![Build  Status](https://travis-ci.org/sambitdash/aimasamples.jl.svg?branch=master)](https://travis-ci.org/sambitdash/aimasamples.jl)
[![Coverage Status](https://coveralls.io/repos/sambitdash/aimasamples.jl/badge.svg?branch=master&service=github)](https://coveralls.io/github/sambitdash/aimasamples.jl?branch=master)
[![codecov.io](http://codecov.io/github/sambitdash/aimasamples.jl/coverage.svg?branch=master)](http://codecov.io/github/sambitdash/aimasamples.jl?branch=master)

This package is developed by the author as a sample program development of
Artificial Intelligence - A Modern Approach 3rd Edition. This is not the
official sample program source for the book. The solution here is just the
author's own interpretation and does not have any dependency or ratification
from any work carried out as part of the official sample code available at:
[aimacode](https://github.com/aimacode). Anyone interested in the official
version may visit the link provided.

The environment is split into two parts. [AIMACore](AIMACore.jl)
and [AIMASamples](AIMASamples.jl). *AIMACore* contains the core
libraries and *AIMASamples* provide examples to test the core library.
Although, the detailed documentation has not been developed, the intent is
to the keep the essence of the algorithm as sacrosanct as possible. Algorithms
are developed as no side effects such that data types can be governed by
underlying applications. Keeping this generalization in mind the typical
data structures are not mapped to a specific implementation but parametrized.
This way a performance sensitive application may use any data structure that
is optimized for the specific application. For someone developing her own
application can use the *AIMACore* library only. There is no reason to add
the additional test libraries.

This sample program library is built as Julia packages which can be loaded into
 the Julia runtime easily.
