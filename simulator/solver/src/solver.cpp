#include    "solver.h"

#include    <QLibrary>

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void Solver::setODEsize(size_t n)
{
    (void) n;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
Solver *loadSolver(QString lib_path)
{
    Solver *solver = nullptr;

    QLibrary lib(lib_path);

    if (lib.load())
    {
        GetSolver getSolver = (GetSolver) lib.resolve("getSolver");

        if (getSolver)
        {
            solver = getSolver();
        }
    }

    return solver;
}
