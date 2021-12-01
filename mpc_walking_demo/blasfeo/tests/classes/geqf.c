// CLASS_GETRF_NOPIVOT
//
void call_routines(struct RoutineArgs *args){

	// unpack args

	// routine call
	//
	BLASFEO(ROUTINE)(
		args->m, args->n,
		args->sA_po, args->ai, args->aj,
		args->sD, args->di, args->dj,
		args->work
		);

	REF(BLASFEO(ROUTINE))(
		args->m, args->n,
		args->rA_po, args->ai, args->aj,
		args->rD, args->di, args->dj,
		args->work
		);

}

void print_routine(struct RoutineArgs *args){
	// unpack args

	printf("%s\n", string(ROUTINE));
	printf(
		"Solving A[%d:%d,%d:%d] = (QR) LQ[%d:%d,%d:%d]\n",
		args->ai, args->m, args->aj,  args->n,
		args->di, args->m, args->dj, args->n
	);

}

void print_routine_matrices(struct RoutineArgs *args)
{
		printf("\nPrint A:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sA_po, args->ai, args->aj, 0, 0, 0);
		print_xmat_debug(args->m, args->n, args->rA_po, args->ai, args->aj, 0, 0, 0);

		printf("\nPrint LU:\n");
		blasfeo_print_xmat_debug(args->m, args->n, args->sD, args->ai, args->aj, 0, 0, 0);
		print_xmat_debug(args->m, args->n, args->rD, args->ai, args->aj, 0, 0, 0);
}


void set_test_args(struct TestArgs *targs)
{
	targs->nis = 9;
	targs->njs = 9;
	targs->nks = 9;

	targs->alphas = 1;
}
